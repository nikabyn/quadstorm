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

const EVENTS_CHANNEL_SIZE: usize = 64;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = init_esp().await;
    init_rtos(peripherals.TIMG0, peripherals.SW_INTERRUPT).await;

    info!("Embassy initialized!");

    let (mut outgoing_events_tx, outgoing_events_rx) =
        spsc_channel!(RemoteRequest, EVENTS_CHANNEL_SIZE).split();
    let (incoming_events_tx, mut incoming_events_rx) =
        spsc_channel!(DroneResponse, EVENTS_CHANNEL_SIZE).split();
    spawner
        .spawn(esp_now_communicate(
            peripherals.WIFI,
            outgoing_events_rx,
            incoming_events_tx,
        ))
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(2000));
    let mut last_ping_sent = None;
    loop {
        let result = select(ticker.next(), incoming_events_rx.receive()).await;
        match result {
            Either::First(_) => {
                if last_ping_sent.replace(Instant::now()).is_some() {
                    warn!("Connection lost!");
                }
                *outgoing_events_tx.send().await = RemoteRequest::Ping;
                outgoing_events_tx.send_done();
            }
            Either::Second(drone_res) => {
                match drone_res {
                    DroneResponse::Pong => {
                        let Some(roundtrip_start) = last_ping_sent.take() else {
                            continue;
                        };
                        info!(
                            "Roundtrip time: {}ms",
                            roundtrip_start.elapsed().as_millis()
                        );
                    }
                    DroneResponse::Log(content) => {
                        info!("Log: {content}");
                    }
                    _ => {
                        error!("Unexpected response: {drone_res:?}");
                    }
                }

                incoming_events_rx.receive_done();
            }
        }
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
