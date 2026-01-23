#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;
use embassy_futures::select::{Either, select};
use esp_alloc as _;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Receiver, Sender};
use embassy_time::{Duration, Instant, Ticker};
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, peripherals::WIFI};
use log::{error, info, warn};

use communication::{DroneResponse, RemoteRequest, spsc_channel};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = init_esp().await;
    init_rtos(peripherals.TIMG0, peripherals.SW_INTERRUPT).await;
    info!("Embassy initialized!");

    let (mut drone_responses, mut remote_requests) = {
        let (remote_tx, remote_rx) = spsc_channel!(RemoteRequest, 64).split();
        let (drone_tx, drone_rx) = spsc_channel!(DroneResponse, 64).split();
        spawner.must_spawn(esp_now_communicate(peripherals.WIFI, remote_rx, drone_tx));
        (drone_rx, remote_tx)
    };

    let mut ticker = Ticker::every(Duration::from_millis(2000));
    let mut last_ping_sent = None;
    loop {
        let result = select(ticker.next(), drone_responses.receive()).await;

        let Either::Second(drone_res) = result else {
            if last_ping_sent.replace(Instant::now()).is_some() {
                warn!("Connection lost!");
            }
            *remote_requests.send().await = RemoteRequest::Ping;
            remote_requests.send_done();
            continue;
        };

        match drone_res {
            DroneResponse::Pong => {
                if let Some(roundtrip_start) = last_ping_sent.take() {
                    info!(
                        "Roundtrip time: {}ms",
                        roundtrip_start.elapsed().as_millis()
                    );
                }
            }
            DroneResponse::Log(content) => {
                info!("Log: {content}");
            }
            _ => {
                error!("Unexpected response: {drone_res:?}");
            }
        }

        drone_responses.receive_done();
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, NoopRawMutex, RemoteRequest>,
    incoming: Sender<'static, NoopRawMutex, DroneResponse>,
) {
    communication::communicate(wifi, outgoing, incoming).await
}

async fn init_esp() -> Peripherals {
    esp_println::logger::init_logger_from_env();

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
