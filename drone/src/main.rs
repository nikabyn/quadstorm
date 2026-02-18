#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;
use defmt_rtt as _;
use esp_backtrace as _;

use alloc::format;
use defmt::{error, info};
use drone::esp_ikarus::bmi323;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0, WIFI};
use esp_hal::timer::timg::TimerGroup;

use common_esp::{mpsc_channel, spsc_channel};
use common_messages::{DroneResponse, RemoteRequest};

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
    let peripherals = init_esp().await;
    init_rtos(peripherals.TIMG0, peripherals.SW_INTERRUPT).await;
    info!("Embassy initialized!");

    // Initialize connection to remote controller
    let (remote_reqests, drone_responses) = {
        let drone = mpsc_channel!(DroneResponse, 64);
        let remote = mpsc_channel!(RemoteRequest, 64);

        spawner.must_spawn(esp_now_communicate(
            peripherals.WIFI,
            drone.receiver(),
            remote.sender(),
        ));

        (remote.receiver(), drone.sender())
    };

    let mut imu_data = {
        info!("IMU init...");

        let poci = peripherals.GPIO5;
        let imu_cs = peripherals.GPIO23;
        //let imu_cs = peripherals.GPIO0;
        let pico = peripherals.GPIO4;
        let sck = peripherals.GPIO16;
        //let sck = peripherals.GPIO9;
        let imu_int1 = peripherals.GPIO22;
        //let imu_int1 = peripherals.GPIO18;
        let imu_spi = peripherals.SPI2;
        let imu_dma = peripherals.DMA_CH0;

        embassy_time::Timer::after_millis(500).await;

        let mut imu = bmi323::BMI323::new(imu_spi, sck, pico, poci, imu_dma, imu_cs, imu_int1);
        if let Err(err) = imu.configure().await {
            error!("{}", format!("{err}"));
        }

        info!("IMU initialized!");

        let (imu_data_tx, imu_data_rx) = spsc_channel!(bmi323::Sample, 2048).split();
        spawner.must_spawn(bmi323::read_imu(imu, imu_data_tx));

        imu_data_rx
    };

    loop {
        if let Ok(remote_req) = remote_reqests.try_receive() {
            match remote_req {
                RemoteRequest::Ping => {
                    drone_responses.send(DroneResponse::Pong).await;
                }
                _ => todo!(),
            }
        }

        if let Some(imu_sample) = imu_data.try_receive() {
            let formatted = format!(
                "gyro={:?}, accl={:?}, time={}",
                imu_sample.gyro, imu_sample.accl, imu_sample.time
            );
            info!("{}", formatted);
            drone_responses.send(DroneResponse::Log(formatted)).await;
            imu_data.receive_done();
        }

        embassy_futures::yield_now().await;
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, CriticalSectionRawMutex, DroneResponse, 64>,
    incoming: Sender<'static, CriticalSectionRawMutex, RemoteRequest, 64>,
) {
    common_esp::communicate(wifi, outgoing, incoming).await;
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
