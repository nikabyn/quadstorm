#![no_std]

extern crate alloc;

use alloc::string::String;

use defmt::{Format, debug, error, info};
use wincode::{SchemaRead, SchemaReadOwned, SchemaWrite};

#[derive(Format, SchemaWrite, SchemaRead)]
#[non_exhaustive]
pub enum RemoteRequest {
    Ping,
    PowerOn,
    PowerOff,
    MotorRpm([u16; 4]),
    Move {
        /// left (-1) to right (+1)
        x: f32,
        /// backwards (-1) to forwards (+1)
        y: f32,
        /// down (-1) to up (+1)
        z: f32,
    },
}

#[derive(Format, SchemaWrite, SchemaRead)]
#[non_exhaustive]
pub enum DroneResponse {
    Pong,
    Log(String),
}
