#![no_std]

extern crate alloc;
use alloc::{boxed::Box, vec::Vec};

use defmt::Format;
use wincode::{SchemaRead, SchemaReadOwned, SchemaWrite};

#[derive(Debug, Format, SchemaWrite, SchemaRead, PartialEq)]
#[non_exhaustive]
pub enum RemoteRequest {
    Ping,
    SetArm(bool),
    ArmConfirm,
    SetThrust(f32),
    SetTarget([f32; 3]),
    SetTune {
        kp: [f32; 3],
        ki: [f32; 3],
        kd: [f32; 3],
    },
}

#[derive(Debug, Format, SchemaWrite, SchemaRead, PartialEq)]
#[non_exhaustive]
pub enum DroneResponse {
    Pong,
    ArmState(bool),
    MotorsState([f32; 4]),
    Log(Box<[u8]>),
}

#[derive(Debug, Format, PartialEq, Eq)]
pub enum FrameDecodeError {
    Corrupted,
    Incomplete,
}

pub struct Frame<T: SchemaWrite<Src = T> + SchemaReadOwned<Dst = T>>(core::marker::PhantomData<T>);

impl<T: SchemaWrite<Src = T> + SchemaReadOwned<Dst = T>> Frame<T> {
    const START: u8 = 0x00;
    const END: u8 = 0xff;

    pub fn encode(value: &T) -> wincode::WriteResult<Box<[u8]>> {
        let base_size = wincode::serialized_size(value)? as usize;
        let mut encoded = Box::new_uninit_slice(base_size);
        wincode::serialize_into(&mut &mut *encoded, &value)?;
        let encoded = unsafe { encoded.assume_init() };

        Ok(Self::escaped(&encoded))
    }

    pub fn decode(data: &[u8]) -> Result<T, FrameDecodeError> {
        let mut unescaped = Self::unescaped(data)?;
        Ok(wincode::deserialize_mut(&mut unescaped).map_err(|_| FrameDecodeError::Corrupted)?)
    }

    fn escaped(data: &[u8]) -> Box<[u8]> {
        // TODO This is not a great way to escape frames,
        //      it sometimes results in frames without a start being interpreted as a valid frame
        //      (when a 0x00 0x00 escape sequence gets cut in half)
        let mut escaped = Vec::with_capacity(data.len() * 2);
        escaped.push(Self::START);
        for &byte in data {
            escaped.push(byte);
            if byte == Self::START || byte == Self::END {
                escaped.push(byte);
            }
        }
        escaped.push(0xff);
        escaped.into_boxed_slice()
    }

    fn unescaped(data: &[u8]) -> Result<Box<[u8]>, FrameDecodeError> {
        if data.first() != Some(&Self::START) {
            return Err(FrameDecodeError::Corrupted);
        }
        if data.len() < 2 {
            return Err(FrameDecodeError::Incomplete);
        }
        if data.last() != Some(&Self::END) {
            return Err(FrameDecodeError::Incomplete);
        }

        let mut result = Vec::with_capacity(data.len());
        let mut i = 1;

        while i < (data.len() - 1) {
            let byte = data[i];

            if byte == Self::START || byte == Self::END {
                if i + 1 >= data.len() {
                    return Err(FrameDecodeError::Incomplete);
                }
                if data[i + 1] != byte {
                    return Err(FrameDecodeError::Corrupted);
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
}

pub struct FrameStreamDecoder<Msg> {
    buffer: [u8; 1024],
    len: usize,
    _msg: core::marker::PhantomData<Msg>,
}

impl<Msg> FrameStreamDecoder<Msg> {
    pub fn new() -> Self {
        Self {
            buffer: [0; 1024],
            len: 0,
            _msg: core::marker::PhantomData,
        }
    }

    /// Read data into internal buffer
    pub fn receive(&mut self, mut f: impl FnMut(&mut [u8]) -> usize) {
        // Read into remaining buffer space
        let read_len = f(&mut self.buffer[self.len..]);
        self.len += read_len;
    }
}

impl<Msg: SchemaWrite<Src = Msg> + SchemaReadOwned<Dst = Msg>> Iterator
    for FrameStreamDecoder<Msg>
{
    type Item = Msg;

    fn next(&mut self) -> Option<Self::Item> {
        let mut processed_up_to = 0;

        let msg = loop {
            if processed_up_to >= self.len {
                // Finished decoding, discard buffer
                self.len = 0;
                processed_up_to = 0;
                break None;
            }

            let Some(start) = self.buffer[processed_up_to..self.len]
                .iter()
                .position(|&b| b == 0x00)
            else {
                // No frame found, discard buffer
                self.len = 0;
                processed_up_to = 0;
                break None;
            };
            let frame_start = processed_up_to + start;

            let Some(end) = self.buffer[frame_start..self.len]
                .iter()
                .position(|&b| b == 0xff)
            else {
                // Incomplete frame, wait for more data
                processed_up_to = frame_start;
                break None;
            };

            let frame_end = frame_start + end;
            let frame = &self.buffer[frame_start..=frame_end];

            match Frame::<Msg>::decode(frame) {
                Ok(msg) => {
                    // Move past current frame, stop decoding
                    processed_up_to = frame_end + 1;
                    break Some(msg);
                }
                Err(FrameDecodeError::Incomplete) => {
                    // Incomplete frame, wait for more data
                    break None;
                }
                Err(FrameDecodeError::Corrupted) => {
                    // Move past current frame, continue decoding
                    processed_up_to = frame_end + 1;
                }
            };
        };

        // Shift remaining data to start of buffer
        if processed_up_to > 0 {
            self.buffer.copy_within(processed_up_to..self.len, 0);
            self.len -= processed_up_to;
        }

        msg
    }
}

#[test]
fn encode_decode_roundtrip() {
    fn roundtrip<
        T: SchemaWrite<Src = T> + SchemaReadOwned<Dst = T> + PartialEq + core::fmt::Debug,
    >(
        v: T,
    ) {
        assert_eq!(Frame::decode(&Frame::encode(&v).unwrap()), Ok(v));
    }

    roundtrip(RemoteRequest::Ping);
    roundtrip(RemoteRequest::ArmConfirm);
    roundtrip(RemoteRequest::SetArm(true));
    roundtrip(RemoteRequest::SetArm(false));
    roundtrip(RemoteRequest::SetTune {
        kp: [0.0, 0.1, 1.0],
        ki: [1.0, 2.0, 100e8],
        kd: [80.0, 0.5, -398.3],
    });

    roundtrip(DroneResponse::Pong);
    roundtrip(DroneResponse::ArmState(true));
    roundtrip(DroneResponse::ArmState(false));
    roundtrip(DroneResponse::Log(Box::from([0, 1, 2, 3])));
}

#[test]
fn stream_decode() {
    use alloc::vec;
    let mut data = Vec::new();

    data.extend_from_slice(
        &Frame::encode(&RemoteRequest::SetTune {
            kp: [0.0, 0.1, 1.0],
            ki: [1.0, 2.0, 100e8],
            kd: [80.0, 0.5, -398.3],
        })
        .unwrap(),
    );
    data.extend_from_slice(&Frame::encode(&RemoteRequest::Ping).unwrap());
    data.extend_from_slice(&Frame::encode(&RemoteRequest::ArmConfirm).unwrap());
    data.extend_from_slice(&Frame::encode(&RemoteRequest::Ping).unwrap());
    data.extend_from_slice(&Frame::encode(&RemoteRequest::ArmConfirm).unwrap());
    data.extend_from_slice(&Frame::encode(&RemoteRequest::SetArm(false)).unwrap());

    data.remove(0);
    data.remove(0);
    data.remove(0);
    data.remove(0);

    let mut decoder = FrameStreamDecoder::<RemoteRequest>::new();
    decoder.receive(|buffer| {
        buffer[..data.len()].copy_from_slice(&data);
        data.len()
    });
    let msgs: Vec<_> = decoder.collect();

    assert_eq!(
        msgs,
        vec![
            RemoteRequest::Ping,
            RemoteRequest::ArmConfirm,
            RemoteRequest::Ping,
            RemoteRequest::ArmConfirm,
            RemoteRequest::SetArm(false)
        ]
    );
}
