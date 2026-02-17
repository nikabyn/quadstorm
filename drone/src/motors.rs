use core::marker::PhantomData;

use defmt::error;
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
            error!("unable to transmit rmt pulse: {:?}", e);
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
        let end = Instant::now().saturating_add(Duration::from_secs(3));
        while Instant::now() <= end {
            self.send_throttles([1000; 4]).await;
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
