#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use alloc::format;
use drone::esp_ikarus::bmi323;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::zerocopy_channel::{Receiver, Sender};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0, WIFI};
use esp_hal::timer::timg::TimerGroup;
use log::info;

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

    // Initialize connection to remote controller
    let (mut remote_reqests, mut drone_responses) = {
        let (drone_tx, drone_rx) = spsc_channel!(DroneResponse, EVENTS_CHANNEL_SIZE).split();
        let (remote_tx, remote_rx) = spsc_channel!(RemoteRequest, EVENTS_CHANNEL_SIZE).split();
        spawner.must_spawn(esp_now_communicate(peripherals.WIFI, drone_rx, remote_tx));

        (remote_rx, drone_tx)
    };

    let mut imu_data = {
        info!("IMU init...");

        let poci = peripherals.GPIO0;
        let imu_cs = peripherals.GPIO1;
        let pico = peripherals.GPIO2;
        let sck = peripherals.GPIO3;
        let imu_int1 = peripherals.GPIO14;
        let imu_spi = peripherals.SPI2;
        let imu_dma = peripherals.DMA_CH0;

        let mut imu = bmi323::BMI323::new(imu_spi, sck, pico, poci, imu_dma, imu_cs, imu_int1);
        imu.configure().await.unwrap();

        info!("IMU initialized!");

        let (imu_data_tx, imu_data_rx) = spsc_channel!(bmi323::Sample, 2048).split();
        spawner.must_spawn(bmi323::read_imu(imu, imu_data_tx));

        imu_data_rx
    };

    loop {
        if let Some(remote_req) = remote_reqests.try_receive() {
            match remote_req {
                RemoteRequest::Ping => {
                    *drone_responses.send().await = DroneResponse::Pong;
                    drone_responses.send_done();
                }
                _ => todo!(),
            }
            remote_reqests.receive_done();
        }

        if let Some(imu_sample) = imu_data.try_receive() {
            let formatted = format!(
                "gyro={:?}, accl={:?}, time={}",
                imu_sample.gyro, imu_sample.accl, imu_sample.time
            );
            *drone_responses.send().await = DroneResponse::Log(formatted);
            imu_data.receive_done();
            drone_responses.send_done();
        }

        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, NoopRawMutex, DroneResponse>,
    incoming: Sender<'static, NoopRawMutex, RemoteRequest>,
) {
    communication::communicate(wifi, outgoing, incoming).await;
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
