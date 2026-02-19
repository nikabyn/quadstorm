use std::fmt::Display;
use std::sync::mpsc;
use std::sync::mpsc::TryRecvError;
use std::thread;
use std::time::Duration;

use anyhow::{Result, anyhow};
use common_messages::{DroneResponse, Frame, FrameStreamDecoder, RemoteRequest};
use defmt_decoder::DecodeError;
use defmt_parser::Level;
use probe_rs::{
    Permissions, Session, probe::list::Lister, rtt::Rtt, rtt::find_rtt_control_block_in_raw_file,
};
use ratatui::DefaultTerminal;
use ratatui::crossterm::event::{self, Event, KeyCode};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::Style;
use ratatui::text::Line;
use ratatui::text::Span;
use ratatui::widgets::{Block, Paragraph};

fn main() -> Result<()> {
    let terminal = ratatui::init();
    let app_result = App::new().start(terminal);
    ratatui::restore();
    app_result
}

#[derive(Clone, Copy)]
enum LogsTab {
    Remote,
    Relay,
    Drone,
}

impl Display for LogsTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Remote => "Remote",
            Self::Relay => "Relay",
            Self::Drone => "Drone",
        })
    }
}

struct App<'a> {
    active_logs_tab: LogsTab,
    log_lines_remote: Vec<Line<'a>>,
    log_lines_relay: Vec<Line<'a>>,
    log_lines_drone: Vec<Line<'a>>,
}

impl<'a> App<'a> {
    fn new() -> Self {
        Self {
            active_logs_tab: LogsTab::Remote,
            log_lines_remote: Vec::new(),
            log_lines_relay: Vec::new(),
            log_lines_drone: Vec::new(),
        }
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let layout = Layout::new(
            Direction::Horizontal,
            [Constraint::Length(40), Constraint::Fill(1)],
        );
        let [controls, logging] = layout.areas(frame.area());

        let line = Line::from("wasd");
        frame.render_widget(line, controls);

        let log_block = Block::bordered().title(self.active_logs_tab.to_string());
        let num_lines = log_block.inner(logging).height;
        let log_lines: Vec<_> = match self.active_logs_tab {
            LogsTab::Remote => &self.log_lines_remote,
            LogsTab::Relay => &self.log_lines_relay,
            LogsTab::Drone => &self.log_lines_drone,
        }
        .iter()
        .rev()
        .take(num_lines as usize)
        .rev()
        .cloned()
        .collect();
        let logging_view = Paragraph::new(log_lines).block(log_block);

        frame.render_widget(logging_view, logging);
    }

    fn start(self, terminal: DefaultTerminal) -> Result<()> {
        let mut args = std::env::args().skip(1);
        let Some(relay_elf_path) = args.next() else {
            return Err(anyhow!("Expected path to relay elf as first argument"));
        };
        let Some(drone_elf_path) = args.next() else {
            return Err(anyhow!("Expected path to drone elf as second argument"));
        };
        let relay_elf = std::fs::read(relay_elf_path)?;
        let drone_elf = std::fs::read(drone_elf_path)?;

        let mut rtt_state = RttState::new(&relay_elf)?;

        let (tx_logs, rx_logs) = mpsc::channel::<(LogsTab, Line)>();
        let (tx_drone_res, rx_drone_res) = mpsc::channel::<DroneResponse>();
        let (tx_remote_req, rx_remote_req) = mpsc::channel::<RemoteRequest>();

        thread::scope(|s| {
            s.spawn(move || {
                let relay_table = defmt_decoder::Table::parse(&relay_elf).unwrap().unwrap();
                let mut relay_logs_decoder = relay_table.new_stream_decoder();

                let drone_table = defmt_decoder::Table::parse(&drone_elf).unwrap().unwrap();
                let mut drone_logs_decoder = drone_table.new_stream_decoder();

                let mut drone_res_decoder = FrameStreamDecoder::<DroneResponse>::new();

                'thread: loop {
                    // Send remote requests
                    'remote_req: loop {
                        let req = match rx_remote_req.try_recv() {
                            Ok(req) => req,
                            Err(TryRecvError::Empty) => break 'remote_req,
                            Err(TryRecvError::Disconnected) => break 'thread,
                        };
                        tx_logs
                            .send((LogsTab::Remote, Line::from(format!("Sending: {:?}", req))))
                            .unwrap();
                        rtt_state.send(0, &Frame::encode(&req).unwrap()).unwrap();
                    }

                    // Receive, decode relay logs
                    let data = rtt_state.receive(0).unwrap();
                    relay_logs_decoder.received(&data);
                    defmt_decode(
                        relay_logs_decoder.as_mut(),
                        &relay_table,
                        LogsTab::Relay,
                        tx_logs.clone(),
                    )
                    .unwrap();

                    // Receive, decode drone logs
                    let data = rtt_state.receive(1).unwrap();
                    drone_logs_decoder.received(&data);
                    defmt_decode(
                        drone_logs_decoder.as_mut(),
                        &drone_table,
                        LogsTab::Drone,
                        tx_logs.clone(),
                    )
                    .unwrap();

                    // Receive, decode drone responses
                    let data = rtt_state.receive(2).unwrap();
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
            });
            self.run(terminal, rx_drone_res, tx_remote_req, rx_logs)
        })
    }

    fn run(
        mut self,
        mut terminal: DefaultTerminal,
        drone_res: mpsc::Receiver<DroneResponse>,
        remote_req: mpsc::Sender<RemoteRequest>,
        logs: mpsc::Receiver<(LogsTab, Line<'a>)>,
    ) -> Result<()> {
        let tick_rate = Duration::from_millis(5);

        loop {
            match logs.try_recv() {
                Ok((tab, line)) => match tab {
                    LogsTab::Remote => &mut self.log_lines_remote,
                    LogsTab::Relay => &mut self.log_lines_relay,
                    LogsTab::Drone => &mut self.log_lines_drone,
                }
                .push(line),
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }
            match drone_res.try_recv() {
                Ok(res) => self
                    .log_lines_remote
                    .push(Line::from(format!("Received: {res:?}"))),
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }

            terminal.draw(|frame| self.draw(frame))?;

            if event::poll(tick_rate)? {
                if let Event::Key(key) = event::read()? {
                    self.log_lines_remote
                        .push(Line::from(format!("Pressed <{}>", key.code.to_string())));
                    match key.code {
                        KeyCode::Char('1') => self.active_logs_tab = LogsTab::Remote,
                        KeyCode::Char('2') => self.active_logs_tab = LogsTab::Relay,
                        KeyCode::Char('3') => self.active_logs_tab = LogsTab::Drone,
                        KeyCode::Char('w') => {}
                        KeyCode::Char('a') => {}
                        KeyCode::Char('s') => {}
                        KeyCode::Char('d') => {}
                        KeyCode::Char('p') => remote_req.send(RemoteRequest::Ping)?,
                        KeyCode::Up => {}
                        KeyCode::Down => {}
                        KeyCode::Esc | KeyCode::Char('q') => break,
                        _ => {}
                    }
                }
            }
        }

        Ok(())
    }
}

struct RttState {
    session: Session,
    rtt: Rtt,
}

impl RttState {
    fn new(elf: &[u8]) -> Result<Self> {
        let probes = Lister::new().list_all();
        let probe = probes[0].open()?;
        let mut session = probe.attach("esp32c6", Permissions::default())?;
        let mut core = session.core(0)?;

        let rtt_pointer = find_rtt_control_block_in_raw_file(elf)?.unwrap();
        let rtt = Rtt::attach_at(&mut core, rtt_pointer)?;
        drop(core);

        Ok(Self { session, rtt })
    }

    fn receive(&mut self, upchannel: usize) -> Result<Box<[u8]>> {
        let Some(input) = self.rtt.up_channel(upchannel) else {
            return Err(anyhow!("Channel {} does not exist", upchannel));
        };
        let mut buffer = vec![0; input.buffer_size()];
        let len = input.read(&mut self.session.core(0)?, &mut buffer)?;
        buffer.truncate(len);
        Ok(buffer.into_boxed_slice())
    }

    fn send(&mut self, downchannel: usize, buffer: &[u8]) -> Result<()> {
        let core = &mut self.session.core(0)?;
        let downchannel = self.rtt.down_channel(downchannel).unwrap();
        let mut written = 0;
        while written != buffer.len() {
            written += downchannel.write(core, buffer)?;
        }
        Ok(())
    }
}

fn defmt_decode(
    decoder: &mut dyn defmt_decoder::StreamDecoder,
    table: &defmt_decoder::Table,
    tab: LogsTab,
    tx: mpsc::Sender<(LogsTab, Line)>,
) -> Result<()> {
    loop {
        match decoder.decode() {
            Ok(frame) => {
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

                tx.send((tab, line)).unwrap();
            }
            Err(DecodeError::Malformed) if table.encoding().can_recover() => {
                // If recovery is possible, skip the current frame and continue with new data.
            }
            Err(DecodeError::Malformed) => {
                return Err(anyhow!(
                    "Unrecoverable error while decoding Defmt \
                    data. Some data may have been lost: {}",
                    DecodeError::Malformed
                ));
            }
            Err(DecodeError::UnexpectedEof) => break,
        }
    }
    Ok(())
}
