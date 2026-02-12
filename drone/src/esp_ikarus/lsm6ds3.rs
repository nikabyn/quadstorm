use defmt::{error, warn};
use embassy_executor::SpawnToken;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::Instant;
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

const READ: u8 = 0x80;
const WRITE: u8 = 0x7f;

pub const FIFO_CTRL1: u8 = 0x06;
pub const FIFO_CTRL2: u8 = 0x07;
pub const FIFO_CTRL3: u8 = 0x08;
pub const FIFO_CTRL4: u8 = 0x09;
pub const FIFO_CTRL5: u8 = 0x0a;

pub const INT1_CTRL: u8 = 0x0d;
pub const INT2_CTRL: u8 = 0x0e;

pub const WHO_AM_I: u8 = 0x0f;
pub const WHO_AM_I_VALUE: u8 = 0x69;

pub const CTRL1_XL: u8 = 0x10;
pub const CTRL2_G: u8 = 0x11;
pub const CTRL3_C: u8 = 0x12;
pub const CTRL4_C: u8 = 0x13;
pub const CTRL5_C: u8 = 0x14;
pub const CTRL6_C: u8 = 0x15;
pub const CTRL7_C: u8 = 0x16;
pub const CTRL8_XL: u8 = 0x17;
pub const CTRL9_XL: u8 = 0x18;
pub const CTRL10_C: u8 = 0x19;
pub const MASTER_CONFIG: u8 = 0x1a;

pub const STATUS: u8 = 0x1e;

pub const FIFO_STATUS1: u8 = 0x3a;
pub const FIFO_STATUS2: u8 = 0x3b;
pub const FIFO_STATUS3: u8 = 0x3c;
pub const FIFO_STATUS4: u8 = 0x3d;

pub const FIFO_DATA_OUT_L: u8 = 0x3e;
pub const FIFO_DATA_OUT_H: u8 = 0x3f;

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

pub struct LSM6DS3 {
    spi: SpiDmaBus<'static, Async>,
    cs: TxPin<'static>,
    int1: Input<'static>,
}

#[derive(Debug, Clone, Copy)]
pub struct FifoStatus {
    pub unread_words: u16,
    pub threshold: bool,
    pub over_run: bool,
    pub full: bool,
    pub empty: bool,
    pub pattern: u16,
}

const SAMPLES_IN_FIFO: usize = 4;
const ENTRIES_PER_SAMPLE: usize = 3;
const WORDS_PER_ENTRY: usize = 3;
const BYTES_PER_WORD: usize = 2;
const FIFO_THRESHOLD: usize = SAMPLES_IN_FIFO * ENTRIES_PER_SAMPLE * WORDS_PER_ENTRY;

#[derive(Debug, Clone, Copy)]
pub struct Sample {
    pub gy: [f32; 3],
    pub xl: [f32; 3],
    pub temp: [f32; 3],
}

#[derive(Debug, Clone, Copy)]
pub enum SampleEvent {
    Ok(Sample),
    Lagged(Sample),
}

#[embassy_executor::task]
async fn read_imu_task(
    mut imu: LSM6DS3,
    mut tx: embassy_sync::zerocopy_channel::Sender<'static, NoopRawMutex, SampleEvent>,
) {
    let mut buf = [0u8; 256];
    let mut leftover_len = 0usize;

    const PATTERNS: u16 = (ENTRIES_PER_SAMPLE * WORDS_PER_ENTRY) as _;

    loop {
        imu.wait_for_data().await;

        while let Ok(FifoStatus {
            unread_words: unread_words @ 1..,
            over_run,
            // pattern is the word index into the current sample
            pattern,

            threshold: _,
            full: _,
            empty: _,
        }) = imu.fifo_status().await
        {
            assert!(pattern < PATTERNS, "lsm6ds3 configuration deviation");

            // First word in buf (buf[0..=1]) should always be the start of a new sample => pattern = 0.
            // Next word in queue has the pattern reported by fifo_status.
            // We read the queue in words of len=2 => leftover_len should always be even.
            //
            // Therefore `leftover_len / 2 == pattern` implies that we are in sync with the queue.
            //
            // When we are not in sync but given condition holds, the queue must
            // have been overrun, which should be indicated by over_run.

            let read_queue = async {
                let len = (buf.len() - (pattern as usize * 2)).min(unread_words as usize * 2);
                let start = pattern as usize * 2;
                let end = start + len;

                imu.read_fifo(&mut buf[..end]).await?;

                Result::<usize, esp_hal::spi::Error>::Ok(end)
            };

            let lag = async {
                if over_run || leftover_len / 2 != pattern as usize {
                    warn!(
                        "fifo lagged, over_run={}, pattern: {}, leftover_len: {} {}",
                        over_run,
                        pattern,
                        leftover_len,
                        leftover_len / 2
                    );

                    // skip to the next full sample
                    return Some(PATTERNS - pattern);
                }

                None
            };

            let (end, lag) = embassy_futures::join::join(read_queue, lag).await;

            let end = match end {
                Ok(end) => end,
                Err(e) => {
                    error!("unable to read IMU data: {:?}", e);
                    // TODO: do something about it
                    break;
                }
            };

            let (words, leftover) = buf[0..end].as_chunks::<BYTES_PER_WORD>();
            assert!(
                leftover.is_empty(),
                "we only read in words, so buf should chunk nicely"
            );

            let (raw_samples, leftover) =
                words.as_chunks::<{ ENTRIES_PER_SAMPLE * WORDS_PER_ENTRY }>();
            let raw_samples_bytes = raw_samples.as_flattened().len();

            for [rx, ry, rz, ax, ay, az, t0, t1, t2] in raw_samples.iter().copied() {
                const MG_PER_LSB: f32 = 0.244; // Scale: 8g
                const MDPS_PER_LSB: f32 = 0.035; // Scale: 1000dps

                let rx = i16::from_le_bytes(rx) as f32 * MDPS_PER_LSB;
                let ry = i16::from_le_bytes(ry) as f32 * MDPS_PER_LSB;
                let rz = i16::from_le_bytes(rz) as f32 * MDPS_PER_LSB;
                let ax = i16::from_le_bytes(ax) as f32 * MG_PER_LSB;
                let ay = i16::from_le_bytes(ay) as f32 * MG_PER_LSB;
                let az = i16::from_le_bytes(az) as f32 * MG_PER_LSB;

                let t0 = (i16::from_le_bytes(t0) as f32 / 256.0) + 25.0;
                let t1 = (i16::from_le_bytes(t1) as f32 / 256.0) + 25.0;
                let t2 = (i16::from_le_bytes(t2) as f32 / 256.0) + 25.0;

                let sample = Sample {
                    gy: [rx, ry, rz],
                    xl: [ax, ay, az],
                    temp: [t0, t1, t2],
                };

                *tx.send().await = match lag {
                    Some(_) => SampleEvent::Lagged(sample),
                    None => SampleEvent::Ok(sample),
                };
                tx.send_done();
            }

            let leftover = leftover.as_flattened();
            leftover_len = leftover.len();
            buf.copy_within(raw_samples_bytes..(raw_samples_bytes + leftover_len), 0);
        }
    }
}

impl LSM6DS3 {
    pub fn start(
        self,
        channel: &'static mut embassy_sync::zerocopy_channel::Channel<NoopRawMutex, SampleEvent>,
    ) -> (
        embassy_sync::zerocopy_channel::Receiver<'static, NoopRawMutex, SampleEvent>,
        SpawnToken<impl Sized>,
    ) {
        let (tx, rx) = channel.split();
        (rx, read_imu_task(self, tx))
    }

    pub async fn fifo_status(&mut self) -> Result<FifoStatus, esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();
        let mut buf = [0; 5];
        self.spi
            .transfer_async(&mut buf, &[READ | FIFO_STATUS1])
            .await?;
        drop(_tx);

        let buf = &buf[1..];
        let status = FifoStatus {
            unread_words: u16::from_le_bytes([buf[0], buf[1] & 0x0f]),
            threshold: buf[1] & (1 << 7) > 0,
            over_run: buf[1] & (1 << 6) > 0,
            full: buf[1] & (1 << 5) > 0,
            empty: buf[1] & (1 << 4) > 0,
            pattern: u16::from_le_bytes([buf[2], buf[3] & 0b11]),
        };

        Ok(status)
    }

    pub async fn read_fifo(&mut self, buf: &mut [u8]) -> Result<(), esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        self.spi.write_async(&[READ | FIFO_DATA_OUT_L]).await?;
        self.spi.read_async(buf).await?;

        Ok(())
    }

    pub async fn wait_for_data(&mut self) {
        self.int1.wait_for_high().await
    }

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
        let who_am_i = self
            .read_register(WHO_AM_I)
            .await
            .map_err(ConfigurationError::Spi)?;

        if who_am_i != WHO_AM_I_VALUE {
            return Err(ConfigurationError::InvalidChip);
        }

        const BDU: u8 = 1 << 6;
        const IF_INC: u8 = 1 << 2;
        const SW_RESET: u8 = 1;

        //// IMU Reset flow
        // Gyro in Power-Down mode
        self.write_register(CTRL2_G, 0)
            .await
            .map_err(ConfigurationError::Spi)?;
        // Accelerometer in High-Performance mode
        self.write_register(CTRL6_C, 0)
            .await
            .map_err(ConfigurationError::Spi)?;
        // Reset
        self.write_register(CTRL3_C, SW_RESET)
            .await
            .map_err(ConfigurationError::Spi)?;

        esp_hal::delay::Delay::new().delay_micros(50);

        let reset_start = Instant::now();
        loop {
            if let Ok(IF_INC) = self
                .read_register(CTRL3_C)
                .await
                .map_err(ConfigurationError::Spi)
            {
                break;
            }

            if Instant::now().duration_since(reset_start).as_secs() >= 1 {
                return Err(ConfigurationError::Timeout);
            }
        }

        // Init CTRL
        self.write_verify_register(CTRL3_C, BDU | IF_INC)
            .await
            .map_err(ConfigurationError::Verification)?;

        const ODR_XL: u8 = 0b1000 << 4;
        const FS_XL: u8 = 0b11 << 2;
        const BW_XL: u8 = 0b10 << 2;
        self.write_verify_register(CTRL1_XL, ODR_XL | FS_XL | BW_XL)
            .await
            .map_err(ConfigurationError::Verification)?;

        const ODR_G: u8 = 0b1000 << 4;
        const FS_G: u8 = 0b10 << 2;
        self.write_verify_register(CTRL2_G, ODR_G | FS_G)
            .await
            .map_err(ConfigurationError::Verification)?;

        const XL_BW_SCAL_ODR: u8 = 1 << 7;
        const FIFO_TEMP_EN: u8 = 1 << 4;
        const I2C_DISABLE: u8 = 1 << 2;
        self.write_verify_register(CTRL4_C, XL_BW_SCAL_ODR | FIFO_TEMP_EN | I2C_DISABLE)
            .await
            .map_err(ConfigurationError::Verification)?;

        const ROUNDING: u8 = 110 << 5;
        self.write_verify_register(CTRL5_C, ROUNDING)
            .await
            .map_err(ConfigurationError::Verification)?;

        // TODO: do we need lpf here?
        // const LPF2_XL_EN: u8 = 1 << 7;
        // const HPCF_XL: u8 = 0b00 << 5;
        // const HP_SLOPE_XL_EN: u8 = 0b1 << 2;
        // self.write_verify_register(CTRL8_XL, LPF2_XL_EN | HPCF_XL | HP_SLOPE_XL_EN)
        //     .await
        //     .map_err(ConfigurationError::Verification)?;

        const ZYX_AXIS_ENABLE: u8 = 0b111 << 3;
        self.write_verify_register(CTRL9_XL, ZYX_AXIS_ENABLE)
            .await
            .map_err(ConfigurationError::Verification)?;
        self.write_verify_register(CTRL10_C, ZYX_AXIS_ENABLE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // Init FIFO
        const FIFO_THRESHOLD_BYTES: [u8; 2] = (FIFO_THRESHOLD as u16).to_le_bytes();
        const FIFO_THRESHOLD_BYTES_L: u8 = FIFO_THRESHOLD_BYTES[0];
        const FIFO_THRESHOLD_BYTES_H: u8 = FIFO_THRESHOLD_BYTES[1] & 0xf;
        self.write_verify_register(FIFO_CTRL1, FIFO_THRESHOLD_BYTES_L)
            .await
            .map_err(ConfigurationError::Verification)?;

        self.write_verify_register(FIFO_CTRL2, FIFO_THRESHOLD_BYTES_H)
            .await
            .map_err(ConfigurationError::Verification)?;

        const DEC_FIFO_GYRO: u8 = 0b001 << 3;
        const DEC_FIFO_XL: u8 = 0b001;
        self.write_verify_register(FIFO_CTRL3, DEC_FIFO_GYRO | DEC_FIFO_XL)
            .await
            .map_err(ConfigurationError::Verification)?;

        const DEC_DS4: u8 = 0b001 << 3;
        self.write_verify_register(FIFO_CTRL4, DEC_DS4)
            .await
            .map_err(ConfigurationError::Verification)?;

        const ODR_FIFO: u8 = 0b1000 << 3;
        const FIFO_MODE: u8 = 0b110;
        self.write_verify_register(FIFO_CTRL5, ODR_FIFO | FIFO_MODE)
            .await
            .map_err(ConfigurationError::Verification)?;

        // Interrupts
        const INT1_FTH: u8 = 1 << 3;
        self.write_verify_register(INT1_CTRL, INT1_FTH)
            .await
            .map_err(ConfigurationError::Verification)?;

        const INT2_FIFO_FULL: u8 = 1 << 5;
        self.write_verify_register(INT2_CTRL, INT2_FIFO_FULL)
            .await
            .map_err(ConfigurationError::Verification)?;

        Ok(())
    }

    async fn read_register(&mut self, reg: u8) -> Result<u8, esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        self.spi.write_async(&[READ | reg]).await?;
        let mut buf = [0];
        self.spi.read_async(&mut buf).await?;

        Ok(buf[0])
    }

    async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), esp_hal::spi::Error> {
        let _tx = self.cs.start_tx();

        self.spi.write_async(&[WRITE & reg, val]).await?;

        Ok(())
    }

    async fn write_verify_register(&mut self, reg: u8, val: u8) -> Result<(), CheckedWriteError> {
        self.write_register(reg, val)
            .await
            .map_err(CheckedWriteError::Spi)?;
        let r = self
            .read_register(reg)
            .await
            .map_err(CheckedWriteError::Spi)?;

        if val != r {
            error!("val {:08b} != {:08b} r", val, r);
            error!("val {:02x} != {:02x} r", val, r);
            return Err(CheckedWriteError::Verification);
        }

        Ok(())
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

    #[error("Chip timed out")]
    Timeout,
}
