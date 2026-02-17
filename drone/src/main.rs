#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;
use defmt_rtt as _;
use drone::{motors, sensor_fusion};
use embassy_time::Duration;
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

use common_esp::{mpmc_channel, spsc_channel};
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
        let drone = mpmc_channel!(DroneResponse, 64);
        let remote = mpmc_channel!(RemoteRequest, 64);

        spawner.must_spawn(esp_now_communicate(
            peripherals.WIFI,
            drone.receiver(),
            remote.sender(),
        ));

        (remote.receiver(), drone.sender())
    };

    let mut imu_data = {
        info!("IMU init...");

        // Drone:
        let poci = peripherals.GPIO5;
        let imu_cs = peripherals.GPIO23;
        let pico = peripherals.GPIO4;
        let sck = peripherals.GPIO16;
        let imu_int1 = peripherals.GPIO22;
        // // Board:
        // let imu_cs = peripherals.GPIO0;
        // let poci = peripherals.GPIO1;
        // let pico = peripherals.GPIO2;
        // let sck = peripherals.GPIO3;
        // let imu_int1 = peripherals.GPIO14;
        let imu_spi = peripherals.SPI2;
        let imu_dma = peripherals.DMA_CH0;

        embassy_time::Timer::after_millis(500).await;

        let mut imu = bmi323::BMI323::new(imu_spi, sck, pico, poci, imu_dma, imu_cs, imu_int1);
        if let Err(err) = imu.configure().await {
            error!("{}", format!("{err}"));
            panic!("{}", err);
        }

        info!("IMU initialized!");

        let (imu_data_tx, imu_data_rx) = spsc_channel!(bmi323::Sample, 32).split();
        spawner.must_spawn(bmi323::read_imu(imu, imu_data_tx));

        imu_data_rx
    };

    let mut motors = motors::Motors::oneshot125(
        peripherals.RMT,
        peripherals.GPIO19,
        (peripherals.GPIO6, peripherals.GPIO20),
    )
    .await;
    motors.arm_oneshot().await;

    let mut fusion = sensor_fusion::ComplementaryFilterFusion::new(
        0.95, [0.0; 3], [0.0; 3], [15.0; 3], [0.0; 3], [0.0; 3],
    );

    let motors_off_until =
        embassy_time::Instant::now().saturating_add(embassy_time::Duration::from_secs(2));
    let mut next_report = embassy_time::Instant::now();

    loop {
        let imu_sample = imu_data.receive().await;
        {
            defmt::debug!(
                "imu: roll={:02}, \tpitch={:02}, \tyaw={:02}, \t\tax={:02}, \tay={:02}, \taz={:02}, \ttime={}",
                imu_sample.gyro[0],
                imu_sample.gyro[1],
                imu_sample.gyro[2],
                imu_sample.accl[0],
                imu_sample.accl[1],
                imu_sample.accl[2],
                imu_sample.time,
            );
            let [roll, pitch, yaw] = fusion.advance(*imu_sample);
            imu_data.receive_done();

            const MOTOR_FRONT_LEFT_IDX: usize = 0;
            const MOTOR_FRONT_LEFT_REV: bool = false;

            const MOTOR_FRONT_RIGHT_IDX: usize = 1;
            const MOTOR_FRONT_RIGHT_REV: bool = false;

            const MOTOR_BACK_RIGHT_IDX: usize = 2;
            const MOTOR_BACK_RIGHT_REV: bool = false;

            const MOTOR_BACK_LEFT_IDX: usize = 3;
            const MOTOR_BACK_LEFT_REV: bool = true;

            let thrust = 0.0;
            let motor_throttles = [
                thrust + roll + pitch - yaw,
                thrust - roll + pitch + yaw,
                thrust - roll - pitch - yaw,
                thrust + roll - pitch + yaw,
            ]
            // .map(|f| f.clamp(-1000.0, 1000.0));
            .map(|f| f.clamp(0.0, 1000.0));

            let mapped_motor_throttles = [
                if MOTOR_FRONT_LEFT_REV {
                    -motor_throttles[MOTOR_FRONT_LEFT_IDX]
                } else {
                    motor_throttles[MOTOR_FRONT_LEFT_IDX]
                },
                if MOTOR_FRONT_RIGHT_REV {
                    -motor_throttles[MOTOR_FRONT_RIGHT_IDX]
                } else {
                    motor_throttles[MOTOR_FRONT_RIGHT_IDX]
                },
                if MOTOR_BACK_RIGHT_REV {
                    -motor_throttles[MOTOR_BACK_RIGHT_IDX]
                } else {
                    motor_throttles[MOTOR_BACK_RIGHT_IDX]
                },
                if MOTOR_BACK_LEFT_REV {
                    -motor_throttles[MOTOR_BACK_LEFT_IDX]
                } else {
                    motor_throttles[MOTOR_BACK_LEFT_IDX]
                },
            ]
            .map(|t| t + 1000.0)
            .map(|t| t as u16);

            if embassy_time::Instant::now() < motors_off_until {
                // some time to let the controller stabilize
                motors.send_throttles([1000; 4]).await;
            } else {
                motors.send_throttles(mapped_motor_throttles).await;
            }

            if embassy_time::Instant::now() >= next_report {
                // info!("dt: {:?}", dt);
                info!("ori: {:?}", fusion.orientation());
                info!("roll: {}, pitch: {}, yaw: {}", roll, pitch, yaw);
                info!("throttles: {:?}\n", mapped_motor_throttles);
                // let formatted = format!(
                //     "ori: {:?}, dt: {}ms, loop_freq: {}Hz",
                //     fusion.orientation(),
                //     dt.as_micros() as f32 / 1_000.0,
                //     1_000_000.0 / (embassy_time::Instant::now() - sample_time).as_micros() as f32
                // );
                // info!("{}", formatted);
                // *drone_responses.send().await = DroneResponse::Log(formatted);
                // drone_responses.send_done();
                next_report = embassy_time::Instant::now().saturating_add(Duration::from_secs(1));
            }
        }

        if let Some(remote_req) = remote_reqests.try_receive() {
            match remote_req {
                RemoteRequest::Ping => {
                    drone_responses.send(DroneResponse::Pong).await;
                }
                _ => todo!(),
            }
        }
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
