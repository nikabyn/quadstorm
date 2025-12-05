#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_ikarus::motors::Motors;
use log::{info, warn};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Initializes and arms the ESCs at GPIO5-GPIO8 and waits for input
// send a numeric ascii value 0..=1000 to change current throttle.
// send "RAMP" to enter a program to ramp the motor up and down.
// send "RAMP" again to leave.
//
// $ echo 100 > /dev/ttyACM0 # set throttle 100
// $ echo 1000 > /dev/ttyACM0 # set max throttle
// $ echo RAMP > /dev/ttyACM0 # enter ramping program

#[esp_rtos::main]
async fn main(_spawner: embassy_executor::Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let p = esp_hal::init(config);

    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_interrupt = esp_hal::interrupt::software::SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let mut motors = Motors::new(p.MCPWM0, p.GPIO5, p.GPIO6, p.GPIO7, p.GPIO8);
    motors.arm().await;

    let (mut rx, _tx) = UsbSerialJtag::new(p.USB_DEVICE).into_async().split();
    let mut buf = [0; 5];
    let mut len = 0;
    let mut ramp = false;

    loop {
        len = rx.drain_rx_fifo(&mut buf[len..]);
        if len > 0 {
            let throttle = match buf {
                [b'R', b'A', b'M', b'P', b'\n'] => {
                    ramp = !ramp;
                    continue;
                }
                [a @ b'0'..=b'9', b'\n', ..] => (a - b'0') as u16,
                [b @ b'0'..=b'9', a @ b'0'..=b'9', b'\n', ..] => {
                    10 * (b - b'0') as u16 + (a - b'0') as u16
                }
                [c @ b'0'..=b'9', b @ b'0'..=b'9', a @ b'0'..=b'9', b'\n', ..] => {
                    100 * (c - b'0') as u16 + 10 * (b - b'0') as u16 + (a - b'0') as u16
                }
                [
                    d @ b'0'..=b'9',
                    c @ b'0'..=b'9',
                    b @ b'0'..=b'9',
                    a @ b'0'..=b'9',
                    b'\n',
                    ..,
                ] => {
                    1000 * (d - b'0') as u16
                        + 100 * (c - b'0') as u16
                        + 10 * (b - b'0') as u16
                        + (a - b'0') as u16
                }
                _ if len == 5 => {
                    warn!("discarding bytes");
                    len = 0;
                    continue;
                }
                _ if buf[0..len].contains(&b'\n') => {
                    warn!("discarding bytes");
                    len = 0;
                    continue;
                }
                _ => continue,
            }
            .clamp(0, 1000);

            let raw_throttle = throttle + 1000;
            motors.set_throttle([raw_throttle; 4]);
            log::info!("throttle: {throttle}, raw_throttle: {raw_throttle}");
        }

        if ramp {
            for throttle in 0..=1000 {
                let raw_throttle = throttle + 1000;
                motors.set_throttle([raw_throttle; 4]);
                log::info!("throttle: {throttle}, raw_throttle: {raw_throttle}");
                Timer::after(Duration::from_millis(2)).await;
            }

            Timer::after(Duration::from_secs(3)).await;

            for throttle in (0..=1000).rev() {
                let raw_throttle = throttle + 1000;
                motors.set_throttle([raw_throttle; 4]);
                log::info!("throttle: {throttle}, raw_throttle: {raw_throttle}");
                Timer::after(Duration::from_millis(2)).await;
            }

            Timer::after(Duration::from_secs(5)).await;
        }
    }
}
