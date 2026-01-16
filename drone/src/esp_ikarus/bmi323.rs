use embassy_executor::SpawnToken;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use esp_hal::{
    Async,
    dma::{DmaChannelFor, DmaRxBuf, DmaTxBuf},
    gpio::{
        Input, InputConfig, InputPin, Output, OutputConfig, OutputPin,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    spi::master::SpiDmaBus,
    time::Rate,
};
use log::{debug, error, info};

const ACC_RANGE: u16 = 0b010 << 4; // +-8g, 4.10 LSB/mg
const MG_PER_LSB: f32 = 1.0 / 4.10;

const GYR_RANGE: u16 = 0b100 << 4; // +-2000deg/s, 16.4 LSB/deg/mg
const DPS_PER_LSB: f32 = 1.0 / 16.4;

const READ: u8 = 0x80;
const WRITE: u8 = 0x7f;

const CHIP_ID: u8 = 0x00;
const CHIP_ID_VALID: u16 = 0x0043;

const ERROR: u8 = 0x01;
const STATUS: u8 = 0x02;

const INT_STATUS2: u8 = 0x0E;
const IO_INT_CTRL: u8 = 0x38;
const INT_MAP2: u8 = 0x3B;

const WORDS_PER_SAMPLE: usize = 3 + // Accel
    3 + // Gyro
    1; // Time
const BYTES_PER_SAMPLE: usize = (WORDS_PER_SAMPLE * 2) as _;

const FIFO_FILL_LEVEL: u8 = 0x15;
const FIFO_DATA: u8 = 0x16;
const FIFO_WATERMARK: u8 = 0x35;
const FIFO_CONF: u8 = 0x36;
const FIFO_CTRL: u8 = 0x37;
const FIFO_FLUSH: u16 = 1;

const ACC_CONF: u8 = 0x20;
const GYR_CONF: u8 = 0x21;

const FEATURE_CTRL: u8 = 0x40;
const FEATURE_IO1: u8 = 0x11;

const CMD: u8 = 0x7E;

pub struct TxPin<'d>(Output<'d>);
pub struct Tx<F: FnOnce()>(Option<F>);

impl<F: FnOnce()> Drop for Tx<F> {
    fn drop(&mut self) {
        if let Some(f) = self.0.take() {
            f()
        }
    }
}

impl<'d> TxPin<'d> {
    fn start_tx(&mut self) -> Tx<impl FnOnce()> {
        self.0.set_low();
        Tx(Some(|| self.0.set_high()))
    }
}

pub struct BMI323 {
    spi: SpiDmaBus<'static, Async>,
    cs: TxPin<'static>,
    int1: Input<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub gy: [f32; 3],
    pub xl: [f32; 3],
    pub time: u16,
}

#[derive(Clone, Copy)]
pub struct FifoStatus {
    unread_words: u16,
    #[allow(unused)]
    fifo_full: bool,
}

#[embassy_executor::task]
async fn read_imu_task(
    mut imu: BMI323,
    mut tx: embassy_sync::zerocopy_channel::Sender<'static, NoopRawMutex, Sample>,
) {
    debug!("[BMI323] beginning to read imu");
    _ = imu.write_register(FIFO_CTRL, FIFO_FLUSH).await;

    let mut buf = [0u8; BYTES_PER_SAMPLE * 64];

    loop {
        imu.wait_for_data().await;

        while let Ok(FifoStatus {
            unread_words: unread_words @ 1..,
            fifo_full: _,
        }) = imu.fifo_status().await
        {
            let len = (unread_words as usize * 2).min(buf.len());
            if let Err(e) = imu.read_fifo(&mut buf).await {
                error!("unable to read from imu: {e:?}");
                // TODO: do something about it
                break;
            }

            let (words, leftover) = buf[0..len].as_chunks::<2>();
            assert!(
                leftover.is_empty(),
                "we only read in words, so buf should chunk nicely"
            );

            let (raw_samples, _) = words.as_chunks::<{ WORDS_PER_SAMPLE }>();

            for [ax, ay, az, rx, ry, rz, time] in raw_samples.iter().copied() {
                let ax = i16::from_le_bytes(ax) as f32 * MG_PER_LSB;
                let ay = i16::from_le_bytes(ay) as f32 * MG_PER_LSB;
                let az = i16::from_le_bytes(az) as f32 * MG_PER_LSB;
                let rx = i16::from_le_bytes(rx) as f32 * DPS_PER_LSB;
                let ry = i16::from_le_bytes(ry) as f32 * DPS_PER_LSB;
                let rz = i16::from_le_bytes(rz) as f32 * DPS_PER_LSB;

                let time = u16::from_le_bytes(time);

                let sample = Sample {
                    gy: [rx, ry, rz],
                    xl: [ax, ay, az],
                    time,
                };

                *tx.send().await = sample;
                tx.send_done();
            }
        }
    }
}

impl BMI323 {
    pub fn new(
        spi: impl esp_hal::spi::master::Instance + 'static,
        sck: impl OutputPin + 'static,
        pico: impl PeripheralOutput<'static>,
        poci: impl PeripheralInput<'static>,
        dma: impl DmaChannelFor<esp_hal::spi::master::AnySpi<'static>>,
        cs: impl OutputPin + 'static,
        int1: impl InputPin + 'static,
    ) -> Self {
        let cs = TxPin(Output::new(
            cs,
            esp_hal::gpio::Level::High,
            OutputConfig::default().with_drive_mode(esp_hal::gpio::DriveMode::PushPull),
        ));

        let int1 = Input::new(
            int1,
            InputConfig::default().with_pull(esp_hal::gpio::Pull::Down),
        );

        let spi = {
            use esp_hal::spi::master::*;

            let (rx_buf, rx_desc, tx_buf, tx_desc) = esp_hal::dma_buffers!(1024);

            Spi::new(
                spi,
                Config::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_mode(esp_hal::spi::Mode::_0)
                    .with_read_bit_order(esp_hal::spi::BitOrder::MsbFirst)
                    .with_write_bit_order(esp_hal::spi::BitOrder::MsbFirst),
            )
            .unwrap()
            .with_sck(sck)
            .with_mosi(pico)
            .with_miso(poci)
            .with_dma(dma)
            .with_buffers(
                DmaRxBuf::new(rx_desc, rx_buf).unwrap(),
                DmaTxBuf::new(tx_desc, tx_buf).unwrap(),
            )
            .into_async()
        };

        Self { spi, cs, int1 }
    }

    pub async fn configure(&mut self) -> Result<(), ConfigurationError> {
        // dummy read to trigger switch to SPI
        _ = self
            .read_register(CHIP_ID)
            .await
            .map_err(ConfigurationError::Spi)?;

        // check chip id
        if CHIP_ID_VALID
            != 0x00ff
                & self
                    .read_register(CHIP_ID)
                    .await
                    .map_err(ConfigurationError::Spi)?
        {
            return Err(ConfigurationError::InvalidChip);
        }

        // reset IMU
        self.write_register(CMD, 0xdeaf)
            .await
            .map_err(ConfigurationError::Spi)?;

        // wait for power up
        let reset = embassy_time::Instant::now();
        loop {
            let status = self
                .read_register(STATUS)
                .await
                .map_err(ConfigurationError::Spi)?;
            if status & 0b11100001 == 0b00000001 {
                break;
            }

            if embassy_time::Instant::now().duration_since(reset).as_secs() > 1 {
                return Err(ConfigurationError::Timeout);
            }
        }

        // dummy read to trigger switch to SPI
        _ = self
            .read_register(CHIP_ID)
            .await
            .map_err(ConfigurationError::Spi)?;

        // check error
        let error = self
            .read_register(ERROR)
            .await
            .map_err(ConfigurationError::Spi)?;
        if error != 0 {
            return Err(ConfigurationError::Internal(error));
        }

        self.self_test_and_calibration().await?;

        // FIFO config
        const FIFO_SAMPLES: u16 = 4;
        const FIFO_WATERMARK_LEVEL: u16 = FIFO_SAMPLES * WORDS_PER_SAMPLE as u16;
        self.write_verify_register(FIFO_WATERMARK, FIFO_WATERMARK_LEVEL)
            .await
            .map_err(ConfigurationError::Verification)?;

        const FIFO_STOP_ON_FULL: u16 = 0; //  Streaming mode
        const FIFO_TIME_EN: u16 = 1 << 8; //  Time in fifo
        const FIFO_ACC_EN: u16 = 1 << 9; //   ACC in fifo
        const FIFO_GYR_EN: u16 = 1 << 10; //  GYR in fifo
        const FIFO_TEMP_EN: u16 = 0 << 11; // Temp in fifo
        self.write_verify_register(
            FIFO_CONF,
            FIFO_STOP_ON_FULL | FIFO_TIME_EN | FIFO_ACC_EN | FIFO_GYR_EN | FIFO_TEMP_EN,
        )
        .await
        .map_err(ConfigurationError::Verification)?;

        const INT1_LVL: u16 = 1; // active high
        const INT1_OD: u16 = 1 << 1; // psuh-pull
        const INT1_EN: u16 = 1 << 2;
        const INT2_LVL: u16 = 1 << 8; // active high
        const INT2_OD: u16 = 1 << 9; // psuh-pull
        const INT2_EN: u16 = 1 << 10;
        self.write_verify_register(
            IO_INT_CTRL,
            INT1_LVL | INT1_OD | INT1_EN | INT2_LVL | INT2_OD | INT2_EN,
        )
        .await
        .map_err(ConfigurationError::Verification)?;

        const FIFO_WATERMARK_INT: u16 = 0b01 << 12; // map to int1
        const FIFO_FULL_INT: u16 = 0b10 << 14; // map to int2
        self.write_verify_register(INT_MAP2, FIFO_WATERMARK_INT | FIFO_FULL_INT)
            .await
            .map_err(ConfigurationError::Verification)?;

        // TODO: turn ODR back up
        // acc config
        const ACC_ODR: u16 = 0b1011; // 0.8kHz
        // const ACC_ODR: u16 = 0b1100; // 1.6kHz
        const ACC_BW: u16 = 0b0 << 7; // ODR/2 TODO: test
        const ACC_AVG: u16 = 0b000 << 8; // Average 0 samples
        const ACC_MODE: u16 = 0b111 << 12; // High performance
        self.write_verify_register(ACC_CONF, ACC_ODR | ACC_RANGE | ACC_BW | ACC_AVG | ACC_MODE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // gyr config
        const GYR_ODR: u16 = 0b1011; // 0.8kHz
        // const GYR_ODR: u16 = 0b1100; // 1.6kHz
        const GYR_BW: u16 = 0b0 << 7; // ODR/2 TODO: test
        const GYR_AVG: u16 = 0b000 << 8; // Average 0 samples
        const GYR_MODE: u16 = 0b111 << 12; // High performance
        self.write_verify_register(GYR_CONF, GYR_ODR | GYR_RANGE | GYR_BW | GYR_AVG | GYR_MODE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // check error
        let error = self
            .read_register(ERROR)
            .await
            .map_err(ConfigurationError::Spi)?;
        if error != 0 {
            return Err(ConfigurationError::Internal(error));
        }
        Ok(())
    }

    async fn self_test_and_calibration(&mut self) -> Result<(), ConfigurationError> {
        // acc config
        const ACC_ODR: u16 = 0b1001; // 200Hz
        const ACC_RANGE: u16 = 0b000 << 4;
        const ACC_BW: u16 = 0b0 << 7;
        const ACC_AVG: u16 = 0b000 << 8;
        const ACC_MODE: u16 = 0b111 << 12;
        self.write_verify_register(ACC_CONF, ACC_ODR | ACC_RANGE | ACC_BW | ACC_AVG | ACC_MODE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // gyr config
        const GYR_ODR: u16 = 0b1001; // 200Hz
        const GYR_RANGE: u16 = 0b000 << 4;
        const GYR_BW: u16 = 0b0 << 7;
        const GYR_AVG: u16 = 0b000 << 8;
        const GYR_MODE: u16 = 0b111 << 12;
        self.write_verify_register(GYR_CONF, GYR_ODR | GYR_RANGE | GYR_BW | GYR_AVG | GYR_MODE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // perform self test and calibration
        const FEATURE_ENGINE_EN: u16 = 1;
        self.write_verify_register(FEATURE_CTRL, FEATURE_ENGINE_EN)
            .await
            .map_err(ConfigurationError::Verification)?;

        // wait for ready to self-test
        loop {
            const STATUS_MASK: u16 = 0b11;
            const STATUS_SHIFT: u16 = 11;
            let status = self
                .read_register(FEATURE_IO1)
                .await
                .map_err(ConfigurationError::Spi)?;

            if (status >> STATUS_SHIFT) & STATUS_MASK == 0 {
                break;
            }
        }

        self.write_register(CMD, 0x100)
            .await
            .map_err(ConfigurationError::Spi)?;

        let self_test = embassy_time::Instant::now();
        loop {
            const STATUS_MASK: u16 = 0b1111;
            const ERROR_SELF_TEST_COMPLETE: u16 = 1 << 4;
            let res = self
                .read_register(FEATURE_IO1)
                .await
                .map_err(ConfigurationError::Spi)?;

            if res & ERROR_SELF_TEST_COMPLETE > 0 {
                let status = res & STATUS_MASK;
                match status {
                    0x5 => {
                        info!("[BMI323] Self-Test Done");
                        break;
                    }
                    e => return Err(ConfigurationError::SelfTest(e as u8)),
                }
            }

            if embassy_time::Instant::now()
                .duration_since(self_test)
                .as_secs()
                > 1
            {
                return Err(ConfigurationError::Timeout);
            }
        }

        // wait for ready to calibrate
        loop {
            const STATUS_MASK: u16 = 0b11;
            const STATUS_SHIFT: u16 = 11;
            let status = self
                .read_register(FEATURE_IO1)
                .await
                .map_err(ConfigurationError::Spi)?;

            if (status >> STATUS_SHIFT) & STATUS_MASK == 0 {
                break;
            }
        }

        // only works when sent twice..
        self.write_register(CMD, 0x101)
            .await
            .map_err(ConfigurationError::Spi)?;
        self.write_register(CMD, 0x101)
            .await
            .map_err(ConfigurationError::Spi)?;

        let calibration = embassy_time::Instant::now();
        loop {
            const STATUS_MASK: u16 = 0b1111;
            const ERROR_SELF_CALIBRATION_COMPLETE: u16 = 1 << 4;
            let res = self
                .read_register(FEATURE_IO1)
                .await
                .map_err(ConfigurationError::Spi)?;

            if res & ERROR_SELF_CALIBRATION_COMPLETE > 0 {
                let status = res & STATUS_MASK;
                match status {
                    0x5 => {
                        info!("[BMI323] Self-Calibration Done");
                        break;
                    }
                    e => return Err(ConfigurationError::SelfCalibration(e as u8)),
                }
            }

            if embassy_time::Instant::now()
                .duration_since(calibration)
                .as_secs()
                > 1
            {
                return Err(ConfigurationError::Timeout);
            }
        }

        self.write_verify_register(FEATURE_CTRL, 0)
            .await
            .map_err(ConfigurationError::Verification)?;

        // // reset gyro and acc
        // self.write_verify_register(ACC_CONF, 0)
        //     .await
        //     .map_err(ConfigurationError::Verification)?;
        // self.write_verify_register(GYR_CONF, 0)
        //     .await
        //     .map_err(ConfigurationError::Verification)?;

        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u16, esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        let mut buf = [READ | reg, 0, 0, 0];
        self.spi.transfer_in_place_async(&mut buf).await?;
        let v = u16::from_le_bytes([buf[2], buf[3]]);

        debug!("[SPI] read(0x{reg:02x}) => 0x{v:04x}");
        Ok(v)
    }

    async fn write_register(&mut self, reg: u8, val: u16) -> Result<(), esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        let [val0, val1] = val.to_le_bytes();
        debug!("[SPI] write(0x{reg:02x}) => 0x{val:04x}");
        self.spi.write_async(&[WRITE & reg, val0, val1]).await?;

        Ok(())
    }

    async fn write_verify_register(&mut self, reg: u8, val: u16) -> Result<(), CheckedWriteError> {
        self.write_register(reg, val)
            .await
            .map_err(CheckedWriteError::Spi)?;
        let r = self
            .read_register(reg)
            .await
            .map_err(CheckedWriteError::Spi)?;

        if val != r {
            error!("val {val:08b} != {r:08b} r");
            error!("val {val:02x} != {r:02x} r");
            return Err(CheckedWriteError::Verification);
        }

        Ok(())
    }

    pub fn start(
        self,
        channel: &'static mut embassy_sync::zerocopy_channel::Channel<NoopRawMutex, Sample>,
    ) -> (
        embassy_sync::zerocopy_channel::Receiver<'static, NoopRawMutex, Sample>,
        SpawnToken<impl Sized>,
    ) {
        let (tx, rx) = channel.split();
        (rx, read_imu_task(self, tx))
    }

    pub async fn fifo_status(&mut self) -> Result<FifoStatus, esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();
        let mut buf = [READ | INT_STATUS2, 0, 0, 0];
        self.spi.transfer_in_place_async(&mut buf).await?;
        drop(_tx);
        let fifo_full = buf[3] & 0x80 > 0;

        let _tx = self.cs.start_tx();
        let mut buf = [READ | FIFO_FILL_LEVEL, 0, 0, 0];
        self.spi.transfer_in_place_async(&mut buf).await?;
        drop(_tx);
        let unread_words = u16::from_le_bytes([buf[2], buf[3]]);

        debug!(
            "[BMI323] fifo_status: full={fifo_full}, unread_words={unread_words}, samples={}",
            unread_words / WORDS_PER_SAMPLE as u16
        );

        Ok(FifoStatus {
            unread_words,
            fifo_full,
        })
    }

    pub async fn read_fifo(&mut self, buf: &mut [u8]) -> Result<(), esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        self.spi.write_async(&[READ | FIFO_DATA, 0]).await?;
        self.spi.read_async(buf).await?;

        Ok(())
    }

    pub async fn wait_for_data(&mut self) {
        self.int1.wait_for_high().await
    }
}

#[derive(thiserror::Error, Debug)]
pub enum CheckedWriteError {
    #[error("Spi error: {0:?}")]
    Spi(esp_hal::spi::Error),

    #[error("Unable to verify register write")]
    Verification,
}

#[derive(thiserror::Error, Debug)]
pub enum ConfigurationError {
    #[error("Spi error: {0:?}")]
    Spi(esp_hal::spi::Error),

    #[error("Verified register write failed")]
    Verification(CheckedWriteError),

    #[error("Unable to verify chip")]
    InvalidChip,

    #[error("Internal error: {0}")]
    Internal(u16),

    #[error("Init error: {0}")]
    Init(u16),

    #[error("Chip timed out")]
    Timeout,

    #[error("Self-Test failed. Status: {0:02x}")]
    SelfTest(u8),

    #[error("Self-Calibration failed: Status: {0:02x}")]
    SelfCalibration(u8),
}
