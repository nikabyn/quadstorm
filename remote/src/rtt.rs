use std::fmt::Display;
use std::marker::PhantomData;
use std::str::FromStr;

use anyhow::{Result as AnyResult, anyhow};
use bevy::ecs::error::Result as BevyResult;
use bevy::ecs::message::{Message, MessageReader, MessageWriter};
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{In, Local};
use bevy::ecs::world::{FromWorld, World};
use bevy::log::error;
use bevy::prelude::Res;
use common_messages::{DroneResponse, Frame, FrameStreamDecoder, RemoteRequest};
use defmt_decoder::DecodeError;
use defmt_parser::Level;
use probe_rs::rtt::{Rtt, find_rtt_control_block_in_raw_file};
use probe_rs::{Permissions, Session, probe::list::Lister};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum LogSource {
    Relay = 1,
    Drone = 2,
}

impl Display for LogSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Relay => "Relay",
            Self::Drone => "Drone",
        })
    }
}

#[derive(Message)]
pub struct RemoteMessage(pub RemoteRequest);
#[derive(Message)]
pub struct DroneMessage(pub DroneResponse);
#[derive(Message)]
pub struct LogMessage(pub LogSource, pub bevy::log::Level, pub String);

pub struct RelayTag;
pub struct DroneTag;

pub fn rtt_communication_system(
    relay_elf: Res<ElfResource<RelayTag>>,
    mut relay_defmt: Local<DefmtState<RelayTag>>,
    mut drone_defmt: Local<DefmtState<DroneTag>>,
    mut rtt_state: Local<Option<RttState>>,
    mut drone_res_decoder: Local<FrameStreamDecoder<DroneResponse>>,
    mut remote_msgs: MessageReader<RemoteMessage>,
    mut drone_msgs: MessageWriter<DroneMessage>,
    mut logs: MessageWriter<LogMessage>,
) -> BevyResult<()> {
    if rtt_state.is_none() {
        *rtt_state = Some(RttState::new(&relay_elf.data)?);
    };
    let rtt_state = rtt_state.as_mut().unwrap();

    // Send remote requests
    for (msg, _) in remote_msgs.par_read() {
        rtt_state.send(0, &Frame::encode(&msg.0)?)?;
    }

    // Receive, decode relay logs
    let data = rtt_state.receive(0)?;
    relay_defmt.decoder.received(&data);
    let lines = relay_defmt.decode_all()?;
    logs.write_batch(lines.into_iter().map(|(level, message)| {
        LogMessage(
            LogSource::Relay,
            FromStr::from_str(level.as_str()).unwrap(),
            message,
        )
    }));

    // Receive, decode drone responses
    let data = rtt_state.receive(1)?;
    drone_res_decoder.receive(|buffer| {
        let len = data.len().min(buffer.len());
        buffer[..len].copy_from_slice(&data[..len]);
        len
    });
    for res in &mut drone_res_decoder {
        if let DroneResponse::Log(data) = &res {
            drone_defmt.decoder.received(data);
        } else {
            drone_msgs.write(DroneMessage(res));
        }
    }
    let lines = drone_defmt.decode_all()?;
    logs.write_batch(lines.into_iter().map(|(level, message)| {
        LogMessage(
            LogSource::Drone,
            FromStr::from_str(level.as_str()).unwrap(),
            message,
        )
    }));

    Ok(())
}

#[derive(Resource)]
pub struct ElfResource<Tag> {
    data: Box<[u8]>,
    defmt_table: &'static defmt_decoder::Table,
    _tag: PhantomData<Tag>,
}

impl<Tag: Send + Sync> ElfResource<Tag> {
    pub fn new(path: String) -> AnyResult<Self> {
        let data = std::fs::read(path)?.into_boxed_slice();
        let defmt_table = defmt_decoder::Table::parse(&data)?
            .ok_or(anyhow!("Could not parse relay defmt table."))?;

        Ok(Self {
            data,
            defmt_table: Box::leak(Box::new(defmt_table)),
            _tag: PhantomData,
        })
    }
}

pub struct RttState {
    session: Session,
    rtt: Rtt,
}

impl RttState {
    pub fn new(elf: &[u8]) -> AnyResult<Self> {
        let probes = Lister::new().list_all();
        let probe = probes
            .get(0)
            .ok_or(anyhow!("No probe connected!"))?
            .open()?;
        let mut session = probe.attach("esp32c6", Permissions::default())?;
        let mut core = session.core(0)?;

        let rtt_pointer = find_rtt_control_block_in_raw_file(elf)?
            .ok_or(anyhow!("No rtt control block in relay elf!"))?;
        let rtt = Rtt::attach_at(&mut core, rtt_pointer)?;
        drop(core);

        Ok(Self { session, rtt })
    }

    pub fn receive(&mut self, upchannel: usize) -> AnyResult<Box<[u8]>> {
        let Some(input) = self.rtt.up_channel(upchannel) else {
            return Err(anyhow!("Channel {} does not exist", upchannel));
        };
        let mut buffer = vec![0; input.buffer_size()];
        let len = input.read(&mut self.session.core(0)?, &mut buffer)?;
        buffer.truncate(len);
        Ok(buffer.into_boxed_slice())
    }

    pub fn send(&mut self, downchannel: usize, buffer: &[u8]) -> AnyResult<()> {
        let core = &mut self.session.core(0)?;
        let downchannel = self
            .rtt
            .down_channel(downchannel)
            .ok_or(anyhow!("No downchannel {downchannel}."))?;
        let mut written = 0;
        while written != buffer.len() {
            written += downchannel.write(core, buffer)?;
        }
        Ok(())
    }
}

pub struct DefmtState<Tag> {
    table: &'static defmt_decoder::Table,
    decoder: Box<dyn defmt_decoder::StreamDecoder + Send + Sync>,
    _tag: PhantomData<Tag>,
}

impl<Tag: Send + Sync + 'static> FromWorld for DefmtState<Tag> {
    fn from_world(world: &mut World) -> Self {
        let table = &world
            .get_resource::<ElfResource<Tag>>()
            .unwrap()
            .defmt_table;

        DefmtState {
            table,
            decoder: table.new_stream_decoder(),
            _tag: PhantomData,
        }
    }
}

impl<Tag> DefmtState<Tag> {
    fn decode_all(&mut self) -> AnyResult<Vec<(defmt_parser::Level, String)>> {
        let mut lines = Vec::new();
        loop {
            match self.decoder.decode() {
                Ok(frame) => lines.push((
                    frame.level().unwrap_or(Level::Trace),
                    frame.display_message().to_string(),
                )),
                Err(DecodeError::Malformed) if self.table.encoding().can_recover() => {
                    // If recovery is possible, skip the current frame and continue with new data.
                }
                Err(DecodeError::Malformed) => Err(anyhow!(
                    "Unrecoverable error while decoding Defmt \
                        data. Some data may have been lost: {}",
                    DecodeError::Malformed
                ))?,
                Err(DecodeError::UnexpectedEof) => break,
            }
        }
        Ok(lines)
    }
}

pub fn log_error_system(In(res): In<BevyResult<()>>) {
    if let Err(err) = res {
        error!("{}", err);
    }
}
