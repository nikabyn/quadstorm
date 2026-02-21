use std::sync::mpsc;
use std::sync::mpsc::TryRecvError;

use anyhow::{Result, anyhow};
use common_messages::DroneResponse;
use common_messages::Frame;
use common_messages::FrameStreamDecoder;
use common_messages::RemoteRequest;
use defmt_decoder::DecodeError;
use defmt_parser::Level;
use probe_rs::{
    Permissions, Session, probe::list::Lister, rtt::Rtt, rtt::find_rtt_control_block_in_raw_file,
};
use ratatui::style::Style;
use ratatui::text::Line;
use ratatui::text::Span;

use crate::logs_tab::LogsTabKind;

pub fn rtt_communicate(
    relay_elf: &[u8],
    drone_elf: &[u8],
    rx_remote_req: &mpsc::Receiver<RemoteRequest>,
    tx_drone_res: &mpsc::Sender<DroneResponse>,
    tx_logs: &mpsc::Sender<(LogsTabKind, Line<'static>)>,
) -> Result<()> {
    let relay_table = defmt_decoder::Table::parse(&relay_elf)?
        .ok_or(anyhow!("Could not parse relay defmt table."))?;
    let mut relay_logs_decoder = relay_table.new_stream_decoder();

    let drone_table = defmt_decoder::Table::parse(&drone_elf)?
        .ok_or(anyhow!("Could not parse drone defmt table."))?;
    let mut drone_logs_decoder = drone_table.new_stream_decoder();

    let mut drone_res_decoder = FrameStreamDecoder::<DroneResponse>::new();

    let mut rtt_state = RttState::new(&relay_elf)?;

    'thread: loop {
        // Send remote requests
        'remote_req: loop {
            let req = match rx_remote_req.try_recv() {
                Ok(req) => req,
                Err(TryRecvError::Empty) => break 'remote_req,
                Err(TryRecvError::Disconnected) => break 'thread,
            };
            tx_logs.send((
                LogsTabKind::Remote,
                Line::from(format!("Sending: {:?}", req)),
            ))?;
            rtt_state.send(0, &Frame::encode(&req)?)?;
        }

        // Receive, decode relay logs
        let data = rtt_state.receive(0)?;
        relay_logs_decoder.received(&data);
        defmt_decode(
            relay_logs_decoder.as_mut(),
            &relay_table,
            LogsTabKind::Relay,
            tx_logs.clone(),
        )?;

        // Receive, decode drone logs
        let data = rtt_state.receive(1)?;
        drone_logs_decoder.received(&data);
        defmt_decode(
            drone_logs_decoder.as_mut(),
            &drone_table,
            LogsTabKind::Drone,
            tx_logs.clone(),
        )?;

        // Receive, decode drone responses
        let data = rtt_state.receive(2)?;
        drone_res_decoder.receive(|buffer| {
            let len = data.len().min(buffer.len());
            buffer[..len].copy_from_slice(&data[..len]);
            len
        });
        for res in &mut drone_res_decoder {
            let Ok(..) = tx_drone_res.send(res) else {
                break 'thread;
            };
        }
    }

    Ok(())
}

pub struct RttState {
    session: Session,
    rtt: Rtt,
}

impl RttState {
    pub fn new(elf: &[u8]) -> Result<Self> {
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

    pub fn receive(&mut self, upchannel: usize) -> Result<Box<[u8]>> {
        let Some(input) = self.rtt.up_channel(upchannel) else {
            return Err(anyhow!("Channel {} does not exist", upchannel));
        };
        let mut buffer = vec![0; input.buffer_size()];
        let len = input.read(&mut self.session.core(0)?, &mut buffer)?;
        buffer.truncate(len);
        Ok(buffer.into_boxed_slice())
    }

    pub fn send(&mut self, downchannel: usize, buffer: &[u8]) -> Result<()> {
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

pub fn defmt_decode(
    decoder: &mut dyn defmt_decoder::StreamDecoder,
    table: &defmt_decoder::Table,
    tab: LogsTabKind,
    tx: mpsc::Sender<(LogsTabKind, Line)>,
) -> Result<()> {
    loop {
        match decoder.decode() {
            Ok(frame) => {
                let line = format_log_line(frame);
                tx.send((tab, line))
                    .map_err(|_| anyhow!("Logs channel closed"))?;
            }
            Err(DecodeError::Malformed) if table.encoding().can_recover() => {
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
    Ok(())
}

fn format_log_line(frame: defmt_decoder::Frame<'_>) -> Line<'static> {
    let mut line = Line::default();

    if let Some(timestamp) = frame.display_timestamp() {
        let span = Span::raw(timestamp.to_string()).style(Style::new().gray());
        line.push_span(span);
    }

    if let Some(level) = frame.level() {
        let style = Style::new().bold();
        let style = match level {
            Level::Trace => style.blue(),
            Level::Debug => style.blue(),
            Level::Info => style.green(),
            Level::Warn => style.yellow(),
            Level::Error => style.red(),
        };
        let span = Span::raw(level.as_str().to_uppercase()).style(style);
        line.push_span("[");
        line.push_span(span);
        line.push_span("] ");
    };

    let message = Span::raw(frame.display_message().to_string());
    line.push_span(message);

    line
}
