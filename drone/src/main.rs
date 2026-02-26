#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;
use core::iter::zip;

use drone::defmt::defmt_data_to_drone_responses;
use drone::{motors, sensor_fusion};
use embassy_futures::select::{Either, select};
use embassy_sync::{channel, zerocopy_channel};
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;

use alloc::format;
use defmt::{error, info, warn};
use drone::esp_ikarus::bmi323;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::{Receiver, Sender};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::{Peripherals, SW_INTERRUPT, TIMG0, WIFI};
use esp_hal::timer::timg::TimerGroup;

use common_esp::{mpmc_channel, spsc_channel};
use common_messages::{DroneResponse, PingTarget, RemoteRequest, Telemetry};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Restart the system on panic
#[unsafe(no_mangle)]
pub fn custom_halt() -> ! {
    esp_hal::system::software_reset()
}

const MOTOR_FRONT_LEFT_IDX: usize = 1;
const MOTOR_FRONT_LEFT_REV: bool = true;

const MOTOR_FRONT_RIGHT_IDX: usize = 2;
const MOTOR_FRONT_RIGHT_REV: bool = false;

const MOTOR_BACK_RIGHT_IDX: usize = 3;
const MOTOR_BACK_RIGHT_REV: bool = true;

const MOTOR_BACK_LEFT_IDX: usize = 0;
const MOTOR_BACK_LEFT_REV: bool = false;

const UNCONFIRMED_ARM_TIME: Duration = Duration::from_millis(500);
const IDLE_THRUST: f32 = 70.0;

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
        spawner.must_spawn(defmt_data_to_drone_responses(drone.sender()));

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
        0.95, [0.0; 3], [0.0; 3], [25.0; 3], [0.0; 3], [10.0; 3],
    );

    let mut telemetry = {
        let (tx, rx) = spsc_channel!(Telemetry, 1).split();
        spawner.must_spawn(log_send_telementry(rx, drone_responses));
        tx
    };

    let mut inputs = {
        let (tx, rx) = spsc_channel!(Input, 16).split();
        spawner.must_spawn(handle_remote_requests(remote_reqests, drone_responses, tx));
        rx
    };

    let mut thrust = 0.0;
    let mut armed = false;
    let mut motors_saturated = false;

    loop {
        if let Some(input) = inputs.try_receive() {
            match input {
                Input::Armed(true) => {
                    armed = true;
                    info!("armed main");
                }
                Input::Armed(false) => {
                    armed = false;
                    info!("disarmed main");
                }
                Input::Target(new_target) => fusion.set_target(*new_target),
                Input::Thrust(new_thrust) => thrust = *new_thrust,
                Input::Tune { kp, ki, kd } => {
                    for i in 0..3 {
                        fusion.pid[i].k_p = kp[i];
                        fusion.pid[i].k_i = ki[i];
                        // reset sum for integral term
                        fusion.pid[i].sum = 0.0;
                        fusion.pid[i].k_d = kd[i];
                    }
                }
            }
            inputs.receive_done();
        }

        let imu_sample = imu_data.receive().await;
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
        let [roll, pitch, yaw] = fusion.advance(*imu_sample, motors_saturated);
        imu_data.receive_done();

        let motor_throttles = [
            thrust - roll - pitch + yaw,
            thrust + roll - pitch - yaw,
            thrust + roll + pitch + yaw,
            thrust - roll + pitch - yaw,
        ];

        let clamped_throttles = motor_throttles
            // .map(|f| f.clamp(-1000.0, 1000.0));
            .map(|f| f.clamp(IDLE_THRUST, 1000.0));

        motors_saturated =
            zip(motor_throttles, clamped_throttles).any(|(raw, clamped)| raw >= clamped);

        let mapped_motor_throttles = map_motor_throttles(clamped_throttles);
        if armed {
            motors.send_throttles(mapped_motor_throttles);
        } else {
            motors.send_throttles([1000; 4]);
        }

        if !armed || thrust < IDLE_THRUST {
            // reset PID integrator when disarmed or low thrust
            fusion.pid.iter_mut().for_each(|pid| pid.sum = 0.0);
        }

        if let Some(msg) = telemetry.try_send() {
            *msg = Telemetry {
                timestamp: Instant::now().as_millis(),
                orientation: fusion.orientation(),
                thrust,
                armed,
                output: [roll, pitch, yaw],
                throttles: mapped_motor_throttles,
            };
            telemetry.send_done();
        };
    }
}

enum Input {
    Thrust(f32),
    Target([f32; 3]),
    Tune {
        kp: [f32; 3],
        ki: [f32; 3],
        kd: [f32; 3],
    },
    Armed(bool),
}

#[embassy_executor::task]
async fn handle_remote_requests(
    remote_requests: channel::Receiver<'static, CriticalSectionRawMutex, RemoteRequest, 64>,
    drone_responses: channel::Sender<'static, CriticalSectionRawMutex, DroneResponse, 64>,
    mut inputs: zerocopy_channel::Sender<'static, NoopRawMutex, Input>,
) -> ! {
    let mut armed = false;
    let mut arm_ticker = Ticker::every(UNCONFIRMED_ARM_TIME);
    let mut thrust = 0.0;

    loop {
        let Either::First(remote_req) = select(remote_requests.receive(), arm_ticker.next()).await
        else {
            if armed {
                warn!("Arm not confirmed in time, disarming...");
                armed = false;
                *inputs.send().await = Input::Armed(false);
                inputs.send_done();
            }

            // Not armed, ignoring
            continue;
        };

        match remote_req {
            RemoteRequest::Ping(target @ PingTarget::Drone, id) => {
                drone_responses.send(DroneResponse::Pong(target, id)).await;
            }
            RemoteRequest::SetArm(true) => {
                if thrust > 10.0 {
                    warn!("drone may not arm when thrust not zero");
                } else {
                    info!("armed");
                    armed = true;
                    arm_ticker.reset();
                    *inputs.send().await = Input::Armed(true);
                    inputs.send_done();
                }

                drone_responses.send(DroneResponse::ArmState(armed)).await;
            }
            RemoteRequest::SetArm(false) => {
                info!("disarmed");
                armed = false;
                *inputs.send().await = Input::Armed(false);
                inputs.send_done();

                drone_responses.send(DroneResponse::ArmState(armed)).await;
            }
            RemoteRequest::ArmConfirm => {
                if armed {
                    arm_ticker.reset();
                } else {
                    warn!("tried to arm confirm unarmed drone");
                }
            }
            RemoteRequest::SetThrust(new_thrust) => {
                thrust = new_thrust;
                *inputs.send().await = Input::Thrust(new_thrust);
                inputs.send_done();
                *inputs.send().await = Input::Thrust(new_thrust);
                inputs.send_done();
            }
            RemoteRequest::SetTarget(target) => {
                *inputs.send().await = Input::Target(target);
                inputs.send_done();
            }
            RemoteRequest::SetTune { kp, ki, kd } => {
                *inputs.send().await = Input::Tune { kp, ki, kd };
                inputs.send_done();
            }
            RemoteRequest::Reset => {
                if armed && thrust > 10.0 {
                    warn!("tried to reset armed and active drone");
                }
                esp_hal::system::software_reset();
            }
            req => warn!("unknown remote request received: {}", req),
        }
    }
}

#[embassy_executor::task]
async fn log_send_telementry(
    mut telemetry: zerocopy_channel::Receiver<'static, NoopRawMutex, Telemetry>,
    drone_responses: channel::Sender<'static, CriticalSectionRawMutex, DroneResponse, 64>,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        ticker.next().await;

        telemetry.clear();
        let received = *telemetry.receive().await;
        drone_responses
            .send(DroneResponse::Telemetry(received))
            .await;
        telemetry.receive_done();
    }
}

fn map_motor_throttles(throttles: [f32; 4]) -> [u16; 4] {
    [
        if MOTOR_FRONT_LEFT_REV {
            -throttles[MOTOR_FRONT_LEFT_IDX]
        } else {
            throttles[MOTOR_FRONT_LEFT_IDX]
        },
        if MOTOR_FRONT_RIGHT_REV {
            -throttles[MOTOR_FRONT_RIGHT_IDX]
        } else {
            throttles[MOTOR_FRONT_RIGHT_IDX]
        },
        if MOTOR_BACK_RIGHT_REV {
            -throttles[MOTOR_BACK_RIGHT_IDX]
        } else {
            throttles[MOTOR_BACK_RIGHT_IDX]
        },
        if MOTOR_BACK_LEFT_REV {
            -throttles[MOTOR_BACK_LEFT_IDX]
        } else {
            throttles[MOTOR_BACK_LEFT_IDX]
        },
    ]
    .map(|t| t + 1000.0)
    .map(|t| t as u16)
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
