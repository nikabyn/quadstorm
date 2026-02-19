#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, peripherals::WIFI};
use rtt_target::{rtt_init, set_defmt_channel};

use common_esp::mpmc_channel;
use common_messages::{DecodeError, DroneResponse, RemoteRequest};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Restart the system on panic
#[unsafe(no_mangle)]
pub fn custom_halt() -> ! {
    esp_hal::system::software_reset()
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let channels = rtt_init! {
        up: {
            0: {
                size: 1024,
                name: "defmt",
            }
            1: {
                size: 1024,
                name: "defmt_drone",
            }
            2: {
                size: 1024,
                name: "drone_res",
            }
        }
        down: {
            0: {
                size: 1024,
                name: "remote_req",
            }
        }
    };
    set_defmt_channel(channels.up.0);

    let peripherals = init_esp().await;
    init_rtos(peripherals.TIMG0, peripherals.SW_INTERRUPT).await;
    info!("Embassy initialized!");

    let (drone_responses, remote_requests) = {
        let remote = mpmc_channel!(RemoteRequest, 64);
        let drone = mpmc_channel!(DroneResponse, 64);

        spawner.must_spawn(esp_now_communicate(
            peripherals.WIFI,
            remote.receiver(),
            drone.sender(),
        ));
        spawner.must_spawn(rtt_communicate(
            channels.up.2,
            channels.down.0,
            remote.sender(),
            drone.receiver(),
        ));

        (drone.receiver(), remote.sender())
    };

    let mut ticker = Ticker::every(Duration::from_millis(2000));
    let mut last_ping_sent = None;

    loop {
        let result = ticker.next().await;

        // TODO: Fix pings
        if last_ping_sent.replace(Instant::now()).is_some() {
            warn!("Connection lost!");
        }
        remote_requests.send(RemoteRequest::Ping).await;
        continue;

        // match drone_res {
        //     DroneResponse::Pong => {
        //         if let Some(roundtrip_start) = last_ping_sent.take() {
        //             info!(
        //                 "Roundtrip time: {}ms",
        //                 roundtrip_start.elapsed().as_millis()
        //             );
        //         }
        //     }
        //     DroneResponse::Log(content) => {
        //         info!("Log: {}", content);
        //     }
        //     _ => {
        //         error!("Unexpected response: {}", drone_res);
        //     }
        // }
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, CriticalSectionRawMutex, RemoteRequest, 64>,
    incoming: Sender<'static, CriticalSectionRawMutex, DroneResponse, 64>,
) {
    common_esp::communicate(wifi, outgoing, incoming).await
}

#[embassy_executor::task]
async fn rtt_communicate(
    mut upchannel: rtt_target::UpChannel,
    mut downchannel: rtt_target::DownChannel,
    outgoing: Sender<'static, CriticalSectionRawMutex, RemoteRequest, 64>,
    incoming: Receiver<'static, CriticalSectionRawMutex, DroneResponse, 64>,
) {
    let mut buffer = [0u8; 1024];
    let mut buffer_len = 0;

    loop {
        // Read from downchannel into remaining buffer space
        if buffer_len < buffer.len() {
            let read_len = downchannel.read(&mut buffer[buffer_len..]);
            buffer_len += read_len;
            if read_len > 0 {
                info!("received: {:?}", buffer[..buffer_len]);
            }
        }

        // Relay all complete frames in the buffer to drone
        let mut processed_up_to = 0;
        loop {
            let Some(start) = buffer[processed_up_to..buffer_len]
                .iter()
                .position(|&b| b == 0x00)
            else {
                // Not a frame, discard data
                buffer_len = 0;
                processed_up_to = 0;
                break;
            };
            let frame_start = processed_up_to + start;

            let Some(end) = buffer[frame_start..buffer_len]
                .iter()
                .position(|&b| b == 0xff)
            else {
                // Incomplete frame, wait for more data
                break;
            };

            let frame_end = frame_start + end;
            let frame = &buffer[frame_start..=frame_end];

            match common_messages::decode::<RemoteRequest>(frame) {
                Ok(req) => {
                    info!("Relaying(to drone): {}", &req);
                    outgoing.send(req).await;
                }
                Err(DecodeError::Corrupted) => {
                    info!("Corrupted frame discarded");
                }
                Err(DecodeError::Incomplete) => {
                    // Incomplete frame, wait for more data
                    break;
                }
            }

            // Move past current frame
            processed_up_to = frame_end + 1;
        }

        // Shift remaining data to start of buffer
        if processed_up_to > 0 {
            buffer.copy_within(processed_up_to..buffer_len, 0);
            buffer_len -= processed_up_to;
        }

        // Relay incoming responses to remote
        while let Ok(res) = incoming.try_receive() {
            info!("Relaying(to remote): {}", res);
            upchannel.write(&common_messages::encode(&res).unwrap());
        }

        embassy_futures::yield_now().await;
    }
}

async fn init_esp() -> Peripherals {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    peripherals
}

async fn init_rtos(timer_group_0: TIMG0<'static>, sw_interrupt: SW_INTERRUPT<'static>) {
    let timg0 = TimerGroup::new(timer_group_0);
    let sw_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(sw_interrupt);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);
}
