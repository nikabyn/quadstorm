use embassy_time::Timer;
use esp_hal::{
    gpio::interconnect::PeripheralOutput,
    mcpwm::{
        McPwm, PeripheralClockConfig, PwmPeripheral,
        operator::{PwmPin, PwmPinConfig},
    },
    time::Rate,
};

/*
 *
 *  TODO: Use OneShot125 at the end of the control loop instead of the current
 *        Servo motor control.
 *
 */

pub struct Motors<PWM: 'static> {
    escs: (
        PwmPin<'static, PWM, 0, true>,
        PwmPin<'static, PWM, 0, false>,
        PwmPin<'static, PWM, 1, true>,
        PwmPin<'static, PWM, 1, false>,
    ),
}

impl<PWM: PwmPeripheral + 'static> Motors<PWM> {
    pub fn new(
        mcpwm: PWM,
        esc_pin1: impl PeripheralOutput<'static>,
        esc_pin2: impl PeripheralOutput<'static>,
        esc_pin3: impl PeripheralOutput<'static>,
        esc_pin4: impl PeripheralOutput<'static>,
    ) -> Self {
        let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
        let mut mcpwm = McPwm::new(mcpwm, clock_cfg);

        mcpwm.operator0.set_timer(&mcpwm.timer0);
        mcpwm.operator1.set_timer(&mcpwm.timer0);
        let config_a = || PwmPinConfig::UP_ACTIVE_HIGH;
        let config_b = || PwmPinConfig::UP_ACTIVE_HIGH;

        let (mut esc_pin1, mut esc_pin2) =
            mcpwm
                .operator0
                .with_pins(esc_pin1, config_a(), esc_pin2, config_b());
        let (mut esc_pin3, mut esc_pin4) =
            mcpwm
                .operator1
                .with_pins(esc_pin3, config_a(), esc_pin4, config_b());

        esc_pin1.set_timestamp(0);
        esc_pin2.set_timestamp(0);
        esc_pin3.set_timestamp(0);
        esc_pin4.set_timestamp(0);

        let timer_clock_cfg = clock_cfg
            .timer_clock_with_frequency(
                19_999,
                esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
                Rate::from_hz(50),
            )
            .expect("motor pwm clock config");
        mcpwm.timer0.start(timer_clock_cfg);

        Self {
            escs: (esc_pin1, esc_pin2, esc_pin3, esc_pin4),
        }
    }

    pub fn set_throttle(&mut self, throttles: [u16; 4]) {
        self.escs.0.set_timestamp(throttles[0]);
        self.escs.1.set_timestamp(throttles[1]);
        self.escs.2.set_timestamp(throttles[2]);
        self.escs.3.set_timestamp(throttles[3]);
    }

    pub async fn arm(&mut self) {
        self.set_throttle([0; 4]);
        Timer::after_millis(1000).await;
        self.set_throttle([1200; 4]);
        Timer::after_millis(1000).await;
        self.set_throttle([1000; 4]);
        Timer::after_millis(1000).await;
    }
}
