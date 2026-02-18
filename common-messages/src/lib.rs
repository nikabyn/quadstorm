#![no_std]

extern crate alloc;
use alloc::{boxed::Box, vec::Vec};

use defmt::Format;
use wincode::{SchemaRead, SchemaReadOwned, SchemaWrite};

#[derive(Debug, Format, SchemaWrite, SchemaRead, PartialEq)]
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

#[derive(Debug, Format, SchemaWrite, SchemaRead, PartialEq)]
#[non_exhaustive]
pub enum DroneResponse {
    Pong,
    Log(Box<[u8]>),
}

pub fn encode<T: SchemaWrite<Src = T>>(value: &T) -> wincode::WriteResult<Box<[u8]>> {
    let base_size = wincode::serialized_size(value)? as usize;
    let mut encoded = Box::new_uninit_slice(base_size);
    wincode::serialize_into(&mut &mut *encoded, &value)?;
    let encoded = unsafe { encoded.assume_init() };

    Ok(escaped(&encoded))
}

#[derive(Debug, Format, PartialEq, Eq)]
pub enum DecodeError {
    Corrupted,
    Incomplete,
}

pub fn decode<T: SchemaReadOwned<Dst = T>>(data: &[u8]) -> Result<T, DecodeError> {
    let mut unescaped = unescaped(data)?;
    Ok(wincode::deserialize_mut(&mut unescaped).map_err(|_| DecodeError::Corrupted)?)
}

fn escaped(data: &[u8]) -> Box<[u8]> {
    let mut escaped = Vec::with_capacity(data.len() * 2);
    escaped.push(0x00);
    for &byte in data {
        escaped.push(byte);
        if byte == 0x00 || byte == 0xFF {
            escaped.push(byte);
        }
    }
    escaped.push(0xff);
    escaped.into_boxed_slice()
}

fn unescaped(data: &[u8]) -> Result<Box<[u8]>, DecodeError> {
    if data.first() != Some(&0x00) {
        return Err(DecodeError::Corrupted);
    }
    if data.len() < 2 {
        return Err(DecodeError::Incomplete);
    }
    if data.last() != Some(&0xff) {
        return Err(DecodeError::Incomplete);
    }

    let mut result = Vec::with_capacity(data.len());
    let mut i = 1;

    while i < (data.len() - 1) {
        let byte = data[i];

        if byte == 0x00 || byte == 0xff {
            if i + 1 >= data.len() {
                return Err(DecodeError::Incomplete);
            }
            if data[i + 1] != byte {
                return Err(DecodeError::Corrupted);
            }
            result.push(byte);
            i += 2;
            continue;
        }

        result.push(byte);
        i += 1;
    }

    Ok(result.into_boxed_slice())
}

#[test]
fn encode_decode_roundtrip() {
    fn roundtrip<
        T: SchemaWrite<Src = T> + SchemaReadOwned<Dst = T> + PartialEq + core::fmt::Debug,
    >(
        v: T,
    ) {
        assert_eq!(decode(&encode(&v).unwrap()), Ok(v));
    }

    roundtrip(RemoteRequest::Ping);
    roundtrip(RemoteRequest::PowerOff);
    roundtrip(RemoteRequest::PowerOn);
    roundtrip(RemoteRequest::MotorRpm([0, 0, 0, 0]));
    roundtrip(RemoteRequest::MotorRpm([0xff, 0xff, 0xff, 0xff]));
    roundtrip(RemoteRequest::MotorRpm([0xff, 0, 0xff, 0]));
    roundtrip(RemoteRequest::Move {
        x: 0.0,
        y: 1.5,
        z: 2.9,
    });

    roundtrip(DroneResponse::Pong);
    roundtrip(DroneResponse::Log(Box::from([0, 1, 2, 3])));
}
