use core::{marker::PhantomData, mem::MaybeUninit};

use embassy_time::{Duration, Instant};
use esp_hal::{
    Async,
    gpio::{Level, Output, OutputConfig, OutputPin, interconnect::PeripheralOutput},
    peripherals::RMT,
    rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator},
    time::Rate,
};
use esp_println::dbg;
use log::error;

pub trait Protocol {
    const RATE: Rate;
    const CLK_DIV: u8;

    fn encode_throttle(throttle: u16) -> ([PulseCode; 17], usize);
}
pub trait Analog: Protocol {}

pub struct DShot300;

impl DShot300 {
    fn encode_frame(value: u16) -> [PulseCode; 17] {
        let value = value << 5;
        let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;

        let frame = (value | crc).reverse_bits();

        let mut pulse = [PulseCode::end_marker(); 17];

        const ONE_HIGH: u16 = 200;
        const ONE_LOW: u16 = 66;
        const ZERO_HIGH: u16 = 100;
        const ZERO_LOW: u16 = 166;

        for i in 0..16 {
            let bit = ((frame >> i) & 0b1) == 0b1;

            let (high, low) = match bit {
                true => (ONE_HIGH, ONE_LOW),
                false => (ZERO_HIGH, ZERO_LOW),
            };

            pulse[i] = PulseCode::new(Level::High, high, Level::Low, low);
        }

        pulse
    }
}

impl Protocol for DShot300 {
    // 80Mhz -> 0.0125us
    // 1 = 200 clock cycles high (2.5us) + 66.4 clock cycles low (0.83us)
    // 0 = 100 clock cycles high (1.25us) + 166.4 clock cycles low (2.08us)
    const RATE: Rate = Rate::from_mhz(80);

    const CLK_DIV: u8 = 1;

    // 11 bits: throttle
    // 1 bit: send back telemetry
    // 4 bit: crc
    fn encode_throttle(throttle: u16) -> ([PulseCode; 17], usize) {
        // convert 0..=1000 range to 48..=2047
        let raw_throttle = (throttle * 2 + 48).min(2047);
        (Self::encode_frame(raw_throttle), 17)
    }
}

pub struct OneShot125;
impl Analog for OneShot125 {}
impl Protocol for OneShot125 {
    // 8 MHz -> 0.125µs
    // throttle = 1000 => pulse of 125µs which is 0 for OneShot125
    // throttle = 2000 => pulse of 250µs which is full throttle for OneShot125
    const RATE: Rate = Rate::from_mhz(8);
    const CLK_DIV: u8 = 1;

    fn encode_throttle(throttle: u16) -> ([PulseCode; 17], usize) {
        let raw_throttle = throttle.min(1000) + 1000;
        let mut pulse = MaybeUninit::uninit().transpose();
        // Low length of 0 is ok because we multiplex that line.
        // For the time we talk to the other esc the line is low anyway,
        // so we don't have to wait for it here.

        pulse[0].write(PulseCode::new(Level::High, raw_throttle, Level::Low, 0));
        pulse[1].write(PulseCode::end_marker());

        (unsafe { pulse.transpose().assume_init() }, 2)
    }
}

pub struct OneShot42;
impl Analog for OneShot42 {}
impl Protocol for OneShot42 {
    // 24 MHz -> ~0.042µs
    // throttle = 1000 => pulse of 42µs which is 0 for OneShot42
    // throttle = 2000 => pulse of 48µs which is full throttle for OneShot42
    const RATE: Rate = Rate::from_mhz(24);
    const CLK_DIV: u8 = 1;

    fn encode_throttle(throttle: u16) -> ([PulseCode; 17], usize) {
        let raw_throttle = throttle.min(1000) + 1000;
        let mut pulse = MaybeUninit::uninit().transpose();
        // Low length of 0 is ok because we multiplex that line.
        // For the time we talk to the other esc the line is low anyway,
        // so we don't have to wait for it here.

        pulse[0].write(PulseCode::new(Level::High, raw_throttle, Level::Low, 0));
        pulse[1].write(PulseCode::end_marker());

        (unsafe { pulse.transpose().assume_init() }, 2)
    }
}

pub struct Motors<Protocol> {
    data: Channel<'static, Async, Tx>,
    mux_slct: [Output<'static>; 2],
    protocol: PhantomData<Protocol>,
}

impl Motors<OneShot125> {
    pub async fn oneshot125(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}

impl Motors<OneShot42> {
    pub async fn oneshot42(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}

impl Motors<DShot300> {
    pub async fn dshot(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}

impl<Proto: Protocol> Motors<Proto> {
    pub async fn new(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        let rmt = Rmt::new(rmt, Proto::RATE).expect("rmt setup").into_async();
        let channel = rmt
            .channel0
            .configure_tx(
                data_pin,
                TxChannelConfig::default()
                    .with_clk_divider(Proto::CLK_DIV)
                    .with_idle_output_level(Level::Low)
                    .with_idle_output(true)
                    .with_carrier_modulation(false)
                    .with_memsize(1),
            )
            .expect("rmt tx channel 0");

        let mux_slct0 = Output::new(mux_slct.0, Level::Low, OutputConfig::default());
        let mux_slct1 = Output::new(mux_slct.1, Level::Low, OutputConfig::default());

        Self {
            data: channel,
            mux_slct: [mux_slct0, mux_slct1],
            protocol: Default::default(),
        }
    }

    async fn send_throttle(&mut self, throttle: u16) {
        let (pulse, len) = Proto::encode_throttle(throttle);
        if let Err(e) = self.data.transmit(&pulse[0..len]).await {
            log::error!("unable to transmit rmt pulse: {e:?}");
        }
    }

    pub async fn send_throttles(&mut self, throttles: [u16; 4]) {
        self.mux_slct[0].set_low();
        self.mux_slct[1].set_low();
        self.send_throttle(throttles[0]).await;

        self.mux_slct[0].set_low();
        self.mux_slct[1].set_high();
        self.send_throttle(throttles[1]).await;

        self.mux_slct[0].set_high();
        self.mux_slct[1].set_low();
        self.send_throttle(throttles[2]).await;

        self.mux_slct[1].set_high();
        self.mux_slct[1].set_high();
        self.send_throttle(throttles[3]).await;
    }
}

impl<Proto: Analog> Motors<Proto> {
    pub async fn arm(&mut self) {
        // Reset
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttles([0; 4]).await;
        }

        // Arming sequence start
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttles([1200; 4]).await;
        }

        // Arming sequence end
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttles([1000; 4]).await;
        }
    }

    pub async fn calibrate(&mut self) {
        // Reset
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttles([0; 4]).await;
        }

        // Arming sequence start
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttles([1200; 4]).await;
        }

        // Set max throttle
        let end = Instant::now().saturating_add(Duration::from_secs(4));
        while Instant::now() <= end {
            self.send_throttles([2000; 4]).await;
        }

        // Set min throttle
        let end = Instant::now().saturating_add(Duration::from_secs(4));
        while Instant::now() <= end {
            self.send_throttles([1000; 4]).await;
        }
    }
}

impl Motors<DShot300> {
    pub async fn arm(&mut self) {
        // Reset
        let end = Instant::now().saturating_add(Duration::from_secs(3));
        while Instant::now() <= end {
            let pulse = DShot300::encode_frame(0);

            self.mux_slct[0].set_low();
            self.mux_slct[1].set_low();
            if let Err(e) = self.data.transmit(&pulse).await {
                log::error!("unable to transmit dshot pulse: {e:?}");
            }

            self.mux_slct[0].set_low();
            self.mux_slct[1].set_high();
            if let Err(e) = self.data.transmit(&pulse).await {
                log::error!("unable to transmit dshot pulse: {e:?}");
            }

            self.mux_slct[0].set_high();
            self.mux_slct[1].set_low();
            if let Err(e) = self.data.transmit(&pulse).await {
                log::error!("unable to transmit dshot pulse: {e:?}");
            }

            self.mux_slct[1].set_high();
            self.mux_slct[1].set_high();
            if let Err(e) = self.data.transmit(&pulse).await {
                log::error!("unable to transmit dshot pulse: {e:?}");
            }
        }
        let end = Instant::now().saturating_add(Duration::from_secs(1));
        while Instant::now() <= end {
            self.send_throttle(0).await;
        }
    }
}
