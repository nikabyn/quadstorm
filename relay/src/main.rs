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

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, peripherals::WIFI};
use rtt_target::{rtt_init, set_defmt_channel};

use common_esp::mpmc_channel;
use common_messages::{DroneResponse, Frame, FrameStreamDecoder, PingTarget, RemoteRequest};

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

    let (drone_res, remote_req) = {
        let remote = mpmc_channel!(RemoteRequest, 64);
        let drone = mpmc_channel!(DroneResponse, 64);

        spawner.must_spawn(esp_now_communicate(
            peripherals.WIFI,
            remote.receiver(),
            drone.sender(),
        ));

        (drone.receiver(), remote.sender())
    };

    let mut req_decoder = FrameStreamDecoder::<RemoteRequest>::default();
    let mut upchannel = channels.up.1;
    let mut downchannel = channels.down.0;

    loop {
        // Relay outgoing requests to drone
        req_decoder.receive(|buffer| downchannel.read(buffer));
        for req in &mut req_decoder {
            if let RemoteRequest::Ping(target @ PingTarget::Relay, ping_id) = &req {
                let res = DroneResponse::Pong(*target, *ping_id);
                upchannel.write(&Frame::encode(&res).unwrap());
                continue;
            }
            info!("Relaying(to drone): {}", &req);
            remote_req.send(req).await;
        }

        // Relay incoming responses to remote
        while let Ok(res) = drone_res.try_receive() {
            info!("Relaying(to remote): {}", res);
            upchannel.write(&Frame::encode(&res).unwrap());
        }

        embassy_futures::yield_now().await;
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
