use core::marker::PhantomData;

use embassy_time::{Duration, Instant};
use esp_hal::{
    Async,
    gpio::{Level, Output, OutputConfig, OutputPin, interconnect::PeripheralOutput},
    peripherals::RMT,
    rmt::{Channel, PulseCode, Rmt, Tx, TxChannelConfig, TxChannelCreator},
    time::Rate,
};

pub trait Protocol {
    const RATE: Rate;
    const CLK_DIV: u8;

    /// transforms a throttle from 0..=2000 into protocol range
    fn throttle_transform(throttle: u16) -> u16;
    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]>;
}

pub trait DShot: Protocol {
    const ONE_HI: u16;
    const ONE_LO: u16;
    const ZERO_HI: u16;
    const ZERO_LO: u16;

    /// Encodes a value into a dshot frame
    ///
    /// 11 bits: throttle
    /// 1 bit: send back telemetry
    /// 4 bit: crc
    fn encode_frame(value: u16, telemetry_request: bool) -> impl AsRef<[PulseCode]> {
        assert!(value < 2048, "dshot max value is 2047");

        let telemetry_request = match telemetry_request {
            true => 1,
            false => 0,
        };
        let data = value << 1 | telemetry_request;
        let crc = (data ^ (data >> 4) ^ (data >> 8)) & 0xf;

        let packet = (data << 4) | crc;
        static mut LAST: u16 = u16::MAX;
        unsafe {
            if LAST != packet {
                log::error!("{value}: {packet:016b} - {packet:04x}");
                LAST = packet;
            }
        }
        let frame = packet.reverse_bits();

        let mut pulse = [PulseCode::end_marker(); 17];
        for i in 0..16 {
            let bit = ((frame >> i) & 0b1) == 0b1;

            let (high, low) = match bit {
                true => (Self::ONE_HI, Self::ONE_LO),
                false => (Self::ZERO_HI, Self::ZERO_LO),
            };

            pulse[i] = PulseCode::new(Level::High, high, Level::Low, low);
        }

        pulse
    }

    fn throttle_transform(throttle: u16) -> u16 {
        throttle.clamp(1, 1999) + 48
    }
}

/// DShot 600
/// Bit length: 1.670µs
/// One High:   1.250µs
/// Zero High:  0.625µs
///
/// With a frequency of 48MHz:
///
/// 1: 60 clock cycles high + 20 clock cycles low
/// 0: 30 clock cycles high + 50 clock cycles low
pub struct DShot600;
impl DShot for DShot600 {
    const ONE_HI: u16 = 60;
    const ONE_LO: u16 = 20;
    const ZERO_HI: u16 = 30;
    const ZERO_LO: u16 = 50;
}
impl Protocol for DShot600 {
    const RATE: Rate = Rate::from_mhz(48);
    const CLK_DIV: u8 = 1;

    fn throttle_transform(throttle: u16) -> u16 {
        <Self as DShot>::throttle_transform(throttle)
    }

    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        Self::encode_frame(value, false)
    }
}

/// DShot 300
/// Bit length: 3.33µs
/// One High:   2.50µs
/// Zero High:  1.25µs
///
/// With a frequency of (48/2)MHz:
///
/// 1: 120 clock cycles high + 40 clock cycles low
/// 0: 60 clock cycles high + 100 clock cycles low
pub struct DShot300;
impl DShot for DShot300 {
    const ONE_HI: u16 = 60;
    const ONE_LO: u16 = 20;
    const ZERO_HI: u16 = 30;
    const ZERO_LO: u16 = 50;
}
impl Protocol for DShot300 {
    const RATE: Rate = Rate::from_mhz(48);
    const CLK_DIV: u8 = 2;

    fn throttle_transform(throttle: u16) -> u16 {
        <Self as DShot>::throttle_transform(throttle)
    }

    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        Self::encode_frame(value, false)
    }
}

/// DShot 150
/// Bit length: 6.67µs
/// One High:   5.00µs
/// Zero High:  2.50µs
///
/// With a frequency of (48/4)MHz:
///
/// 1: 120 clock cycles high + 40 clock cycles low
/// 0: 60 clock cycles high + 100 clock cycles low
pub struct DShot150;
impl DShot for DShot150 {
    const ONE_HI: u16 = 60;
    const ONE_LO: u16 = 20;
    const ZERO_HI: u16 = 30;
    const ZERO_LO: u16 = 50;
}
impl Protocol for DShot150 {
    const RATE: Rate = Rate::from_mhz(48);
    const CLK_DIV: u8 = 4;

    fn throttle_transform(throttle: u16) -> u16 {
        <Self as DShot>::throttle_transform(throttle)
    }

    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        Self::encode_frame(value, false)
    }
}

pub trait OneShot: Protocol {
    fn throttle_transform(throttle: u16) -> u16 {
        (throttle / 2).min(1000) + 1000
    }

    fn encode_oneshot_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        // Low length of 0 is ok because we multiplex that line.
        // For the time we talk to the other esc the line is low anyway,
        // so we don't have to wait for it here.
        [
            PulseCode::new(Level::High, value, Level::Low, 0),
            PulseCode::end_marker(),
        ]
    }
}

pub struct OneShot125;
impl OneShot for OneShot125 {}
impl Protocol for OneShot125 {
    // 8 MHz -> 0.125µs
    // throttle = 1000 => pulse of 125µs which is 0 for OneShot125
    // throttle = 2000 => pulse of 250µs which is full throttle for OneShot125
    const RATE: Rate = Rate::from_mhz(8);
    const CLK_DIV: u8 = 1;

    fn throttle_transform(throttle: u16) -> u16 {
        <Self as OneShot>::throttle_transform(throttle)
    }

    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        Self::encode_oneshot_pulse(value)
    }
}

pub struct OneShot42;
impl OneShot for OneShot42 {}
impl Protocol for OneShot42 {
    // 24 MHz -> ~0.042µs
    // throttle = 1000 => pulse of 42µs which is 0 for OneShot42
    // throttle = 2000 => pulse of 48µs which is full throttle for OneShot42
    const RATE: Rate = Rate::from_mhz(24);
    const CLK_DIV: u8 = 1;

    fn throttle_transform(throttle: u16) -> u16 {
        <Self as OneShot>::throttle_transform(throttle)
    }

    fn encode_pulse(value: u16) -> impl AsRef<[PulseCode]> {
        Self::encode_oneshot_pulse(value)
    }
}

pub struct Motors<Protocol> {
    data: Channel<'static, Async, Tx>,
    mux_slct: [Output<'static>; 2],
    protocol: PhantomData<Protocol>,
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

        let mux_slct0 = Output::new(mux_slct.1, Level::Low, OutputConfig::default());
        let mux_slct1 = Output::new(mux_slct.0, Level::Low, OutputConfig::default());

        Self {
            data: channel,
            mux_slct: [mux_slct0, mux_slct1],
            protocol: Default::default(),
        }
    }

    async fn send_esc_value(&mut self, value: u16) {
        let pulse = Proto::encode_pulse(value);
        if let Err(e) = self.data.transmit(pulse.as_ref()).await {
            log::error!("unable to transmit rmt pulse: {e:?}");
        }
    }

    pub async fn send_esc_values(&mut self, values: [u16; 4]) {
        self.mux_slct[0].set_low();
        self.mux_slct[1].set_low();
        self.send_esc_value(values[0]).await;

        self.mux_slct[0].set_low();
        self.mux_slct[1].set_high();
        self.send_esc_value(values[1]).await;

        self.mux_slct[0].set_high();
        self.mux_slct[1].set_low();
        self.send_esc_value(values[2]).await;

        self.mux_slct[1].set_high();
        self.mux_slct[1].set_high();
        self.send_esc_value(values[3]).await;
    }

    pub async fn send_throttles(&mut self, throttles: [u16; 4]) {
        self.send_esc_values(throttles.map(Proto::throttle_transform))
            .await
    }
}

impl<Proto: OneShot> Motors<Proto> {
    pub async fn arm_oneshot(&mut self) {
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

impl<Proto: DShot> Motors<Proto> {
    pub async fn arm_dshot(&mut self) {
        // Reset
        let end = Instant::now().saturating_add(Duration::from_millis(420));
        while Instant::now() <= end {
            self.send_esc_values([0; 4]).await;
        }

        // Arm
        let end = Instant::now().saturating_add(Duration::from_millis(420));
        while Instant::now() <= end {
            self.send_esc_values([48; 4]).await;
        }

        // Zero Throttle
        let end = Instant::now().saturating_add(Duration::from_millis(420));
        while Instant::now() <= end {
            self.send_throttles([0; 4]).await;
        }
    }
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

impl Motors<DShot600> {
    pub async fn dshot600(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}

impl Motors<DShot300> {
    pub async fn dshot300(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}

impl Motors<DShot150> {
    pub async fn dshot150(
        rmt: RMT<'static>,
        data_pin: impl PeripheralOutput<'static>,
        mux_slct: (impl OutputPin + 'static, impl OutputPin + 'static),
    ) -> Self {
        Self::new(rmt, data_pin, mux_slct).await
    }
}
