#![no_std]
pub mod esp_ikarus;
pub mod sensor_fusion;

pub trait ImuSample {
    fn gyro(&self) -> [f32; 3];
    fn accel(&self) -> [f32; 3];
}
