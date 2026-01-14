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

use common::{QuadcopterResponse, RemoteRequest};
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, Receiver, Sender},
};
use embassy_time::{Duration, Ticker};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, peripherals::WIFI};
use log::info;

use remote_esp::{esp_now, make_static};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const EVENTS_CHANNEL_SIZE: usize = 64;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let outgoing_events =
        make_static!(Channel<NoopRawMutex, RemoteRequest, EVENTS_CHANNEL_SIZE>, Channel::new());
    let incoming_events =
        make_static!(Channel<NoopRawMutex, RemoteRequest, EVENTS_CHANNEL_SIZE>, Channel::new());

    spawner
        .spawn(esp_now_communicate(
            peripherals.WIFI,
            outgoing_events.receiver(),
            incoming_events.sender(),
        ))
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(2000));
    loop {
        ticker.next().await;

        outgoing_events.send(RemoteRequest::Ping).await;
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, NoopRawMutex, RemoteRequest, EVENTS_CHANNEL_SIZE>,
    incoming: Sender<'static, NoopRawMutex, RemoteRequest, EVENTS_CHANNEL_SIZE>,
) {
    esp_now::communicate(wifi, outgoing, incoming).await
}
