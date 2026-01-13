#![no_std]

extern crate alloc;

use alloc::string::String;

#[derive(Debug, bincode::Encode, bincode::Decode)]
pub enum RemoteRequest {
    Ping,
    PowerOn,
    PowerOff,
    Move {
        /// left (-1) to right (+1)
        x: f32,
        /// backwards (-1) to forwards (+1)
        y: f32,
        /// down (-1) to up (+1)
        z: f32,
    },
}

#[derive(Debug, bincode::Encode, bincode::Decode)]
pub enum QuadcopterResponse {
    Pong,
    Log(String),
}
