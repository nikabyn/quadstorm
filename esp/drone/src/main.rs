#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::{GPIO4, GPIO5, I2C0, Peripherals, SW_INTERRUPT, TIMG0, WIFI};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{i2c::master, time::Rate};
use esp_println::println;
use log::info;

use communication::{QuadcopterResponse, RemoteRequest, static_channel};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SENSOR_ADDR: u8 = 0x6B;
const EVENTS_CHANNEL_SIZE: usize = 64;

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = init_esp().await;
    init_rtos(peripherals.TIMG0, peripherals.SW_INTERRUPT).await;
    info!("Embassy initialized!");

    let outgoing_events = static_channel!(QuadcopterResponse, EVENTS_CHANNEL_SIZE);
    let incoming_events = static_channel!(RemoteRequest, EVENTS_CHANNEL_SIZE);
    spawner
        .spawn(esp_now_communicate(
            peripherals.WIFI,
            outgoing_events.receiver(),
            incoming_events.sender(),
        ))
        .unwrap();

    // spawner
    //     .spawn(imu(peripherals.I2C0, peripherals.GPIO4, peripherals.GPIO5))
    //     .unwrap();

    loop {
        let remote_req = incoming_events.receive().await;
        match remote_req {
            RemoteRequest::Ping => {
                outgoing_events.send(QuadcopterResponse::Pong).await;
            }
            _ => {}
        }
    }
}

#[embassy_executor::task]
async fn esp_now_communicate(
    wifi: WIFI<'static>,
    outgoing: Receiver<'static, NoopRawMutex, QuadcopterResponse, EVENTS_CHANNEL_SIZE>,
    incoming: Sender<'static, NoopRawMutex, RemoteRequest, EVENTS_CHANNEL_SIZE>,
) {
    communication::communicate(wifi, outgoing, incoming).await;
}

#[embassy_executor::task]
async fn imu(i2c: I2C0<'static>, sda: GPIO4<'static>, scl: GPIO5<'static>) {
    let c = master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = master::I2c::new(i2c, c)
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

    let mut read_buffer = [0u8];
    if i2c
        .write_read(SENSOR_ADDR, &[0x0F], &mut read_buffer)
        .is_ok()
    {
        info!("READ buffer: {:X?}", read_buffer);
    }

    // Enable accelerometer (CTRL1_XL)
    i2c.write(SENSOR_ADDR, &[0x10, 0x60]).unwrap();
    // Enable gyro (CTRL2_G)
    i2c.write(SENSOR_ADDR, &[0x11, 0x60]).unwrap();

    // CTRL3_C: BDU=1 (bit 6), IF_INC=1 (bit 2)
    i2c.write(SENSOR_ADDR, &[0x12, 0b0100_0100]).unwrap();

    let comp = get_comp_values({
        let mut data = [0u8; 12];
        let mut comp_data = [[0i16; 6]; 400];
        for i in 0..400 {
            i2c.write_read(SENSOR_ADDR, &[0x22], &mut data).unwrap();
            comp_data[i] = process_imu_data(data);
        }
        comp_data
    });
    info!("sensor compensation values: {:?}", comp);

    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        let mut st = [0u8; 1];
        i2c.write_read(SENSOR_ADDR, &[0x1E], &mut st).unwrap();
        println!("{:X?}", &st);
        if st[0] & 0b1 != 0 && st[0] & 0b10 != 0 {
            let mut data = [0u8; 12];
            if let Err(e) = i2c.write_read(SENSOR_ADDR, &[0x22], &mut data) {
                println!("Error: {:?}", e);
                continue;
            }
            let rdata = process_imu_data(data);
            let gz = (rdata[0] - comp[0]) as f64 * 0.00875;
            let gx = (rdata[1] - comp[1]) as f64 * 0.00875;
            let gy = (rdata[2] - comp[2]) as f64 * 0.00875;
            let ax = (rdata[3] - comp[3]) as f64 * 0.000061;
            let ay = (rdata[4] - comp[4]) as f64 * 0.000061;
            let az = (rdata[5] - comp[5]) as f64 * 0.000061;

            println!(
                "Accel[g]: ({:.2}, {:.2}, {:.2}) | Gyro[dps]: ({:.2}, {:.2}, {:.2})",
                ax, ay, az, gx, gy, gz
            );
        }
        ticker.next().await;
    }
}

fn process_imu_data(data: [u8; 12]) -> [i16; 6] {
    let mut res = [0i16; 6];
    for i in 0..6 {
        res[i] = i16::from_le_bytes([data[i * 2], data[i * 2 + 1]]);
    }
    res
}

fn get_comp_values(data: [[i16; 6]; 400]) -> [i16; 6] {
    let sums = data.iter().fold([0i64; 6], |mut acc, elem| {
        for i in 0..6 {
            acc[i] += elem[i] as i64;
        }
        acc
    });
    sums.map(|x| (x / 400) as i16)
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
