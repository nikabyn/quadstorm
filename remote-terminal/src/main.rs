use std::fmt::Display;
use std::sync::mpsc;
use std::sync::mpsc::TryRecvError;
use std::thread;
use std::time::Duration;

use anyhow::{Result, anyhow};
use common_messages::{DroneResponse, RemoteRequest};
use defmt_decoder::DecodeError;
use defmt_parser::Level;
use probe_rs::Core;
use probe_rs::{
    Permissions, Session, probe::list::Lister, rtt::Rtt, rtt::find_rtt_control_block_in_raw_file,
};
use ratatui::DefaultTerminal;
use ratatui::Frame;
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

enum LogginTab {
    Relay,
    Drone,
}

impl Display for LogginTab {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Relay => "Relay",
            Self::Drone => "Drone",
        })
    }
}

struct App<'a> {
    logging_lines: Vec<Line<'a>>,
    logging_tab: LogginTab,
}

impl<'a> App<'a> {
    fn new() -> Self {
        Self {
            logging_lines: Vec::new(),
            logging_tab: LogginTab::Drone,
        }
    }

    fn draw(&self, frame: &mut Frame) {
        let layout = Layout::new(
            Direction::Horizontal,
            [Constraint::Length(40), Constraint::Fill(1)],
        );
        let [controls, logging] = layout.areas(frame.area());

        let line = Line::from("wasd");
        frame.render_widget(line, controls);

        let logging_block = Block::bordered().title(self.logging_tab.to_string());
        let num_lines = logging_block.inner(logging).height;
        let logging_lines: Vec<_> = self
            .logging_lines
            .iter()
            .rev()
            .take(num_lines as usize)
            .rev()
            .cloned()
            .collect();
        let logging_view = Paragraph::new(logging_lines).block(logging_block);

        frame.render_widget(logging_view, logging);
    }

    fn start(self, terminal: DefaultTerminal) -> Result<()> {
        let Some(elf_path) = std::env::args().skip(1).next() else {
            return Err(anyhow!("Expected path to elf as first argument"));
        };
        let elf = std::fs::read(elf_path)?;

        let mut rtt_state = RttState::new(&elf)?;

        let (tx_logs_relay, rx_logs_relay) = mpsc::channel::<Line>();
        let (tx_logs_drone, rx_logs_drone) = mpsc::channel::<Line>();
        let (tx_drone_res, rx_drone_res) = mpsc::channel::<RemoteRequest>();
        let (tx_remote_req, rx_remote_req) = mpsc::channel::<DroneResponse>();

        thread::scope(|s| {
            s.spawn(move || {
                let table = defmt_decoder::Table::parse(&elf).unwrap().unwrap();
                let mut decoder = table.new_stream_decoder();

                loop {
                    let core = &mut rtt_state.session.core(0).unwrap();
                    let rtt = &mut rtt_state.rtt;
                    receive(0, core, rtt, decoder.as_mut()).unwrap();
                    decode(decoder.as_mut(), &table, tx_logs_relay.clone()).unwrap();
                }
            });
            self.run(terminal, rx_logs_relay, rx_logs_drone)
        })
    }

    fn run(
        mut self,
        mut terminal: DefaultTerminal,
        logs_relay: mpsc::Receiver<Line<'a>>,
        logs_drone: mpsc::Receiver<Line<'a>>,
    ) -> Result<()> {
        let tick_rate = Duration::from_millis(5);

        loop {
            match logs_relay.try_recv() {
                Ok(line) => self.logging_lines.push(line),
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }

            terminal.draw(|frame| self.draw(frame))?;

            if event::poll(tick_rate)? {
                if let Event::Key(key) = event::read()? {
                    match key.code {
                        KeyCode::Char('1') => self.logging_tab = LogginTab::Drone,
                        KeyCode::Char('2') => self.logging_tab = LogginTab::Relay,
                        KeyCode::Char('w') => {}
                        KeyCode::Char('a') => {}
                        KeyCode::Char('s') => {}
                        KeyCode::Char('d') => {}
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
}

fn receive(
    number: usize,
    core: &mut Core<'_>,
    rtt: &mut Rtt,
    decoder: &mut dyn defmt_decoder::StreamDecoder,
) -> Result<()> {
    let Some(input) = rtt.up_channel(number) else {
        return Err(anyhow!("Channel {} does not exist", number));
    };
    // SAFETY the uninitialized data is never read
    let mut buffer = unsafe { Box::new_uninit_slice(input.buffer_size()).assume_init() };
    let count = input.read(core, &mut buffer)?;
    decoder.received(&buffer[..count]);
    Ok(())
}

fn decode(
    decoder: &mut dyn defmt_decoder::StreamDecoder,
    table: &defmt_decoder::Table,
    tx: mpsc::Sender<Line>,
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

                tx.send(line).unwrap();
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
