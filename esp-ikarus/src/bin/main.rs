#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::lazy_lock::LazyLock;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_ikarus::lsm6ds3;
use log::info;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let miso = p.GPIO0;
    let imu_cs = p.GPIO1;
    let mosi = p.GPIO2;
    let sck = p.GPIO3;
    let imu_int1 = p.GPIO14;
    let imu_spi = p.SPI2;
    let imu_dma = p.DMA_CH0;

    // SPI
    log::info!("IMU init..");

    let mut imu = lsm6ds3::LSM6DS3::new(imu_spi, sck, mosi, miso, imu_dma, imu_cs, imu_int1);
    imu.configure().await.unwrap();

    log::info!("IMU ready");

    let (imu_rx, imu_task) = {
        static mut CHANNEL_BUF: MaybeUninit<[lsm6ds3::SampleEvent; 8]> =
            core::mem::MaybeUninit::uninit();
        static mut CHANNEL: LazyLock<
            embassy_sync::zerocopy_channel::Channel<'static, NoopRawMutex, lsm6ds3::SampleEvent>,
        > = LazyLock::new(|| {
            #[allow(static_mut_refs)]
            embassy_sync::zerocopy_channel::Channel::new(unsafe { CHANNEL_BUF.assume_init_mut() })
        });

        #[allow(static_mut_refs)]
        imu.start(unsafe { CHANNEL.get_mut() })
    };

    spawner.must_spawn(imu_consumer(imu_rx));
    spawner.must_spawn(imu_task);

    loop {
        log::info!("still alive");
        Timer::after(Duration::from_secs(10)).await;
    }
}

#[embassy_executor::task]
async fn imu_consumer(
    mut rx: embassy_sync::zerocopy_channel::Receiver<'static, NoopRawMutex, lsm6ds3::SampleEvent>,
) {
    loop {
        let sample = *rx.receive().await;
        rx.receive_done();
        match sample {
            lsm6ds3::SampleEvent::Ok(sample) => log::info!("{sample:0.4?}"),
            lsm6ds3::SampleEvent::Lagged(sample) => log::warn!("{sample:?}"),
        }
    }
}
