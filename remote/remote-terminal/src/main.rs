use std::fmt::Display;

use anyhow::{Result, anyhow};
use defmt_decoder::DecodeError;
use defmt_parser::Level;
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
    let app = App::new()?;

    let terminal = ratatui::init();
    let app_result = app.run(terminal);
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
    session: Session,
    rtt: Rtt,
    defmt_table: defmt_decoder::Table,
    defmt_buffer: Vec<u8>,
    logging_lines: Vec<Line<'a>>,
    logging_tab: LogginTab,
}

impl App<'_> {
    fn new() -> Result<Self> {
        let Some(elf_path) = std::env::args().skip(1).next() else {
            return Err(anyhow!("Expected path to elf as first argument"));
        };
        let elf = std::fs::read(elf_path)?;

        let probes = Lister::new().list_all();
        let probe = probes[0].open()?;
        let mut session = probe.attach("esp32c6", Permissions::default())?;
        let mut core = session.core(0)?;

        let rtt_pointer = find_rtt_control_block_in_raw_file(&elf)?.unwrap();
        let rtt = Rtt::attach_at(&mut core, rtt_pointer)?;
        drop(core);

        let defmt_table = defmt_decoder::Table::parse(&elf)?.unwrap();

        Ok(Self {
            session,
            rtt,
            defmt_table,
            defmt_buffer: Vec::new(),
            logging_lines: Vec::new(),
            logging_tab: LogginTab::Drone,
        })
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
        let logging_view = Paragraph::new(self.logging_lines.clone()).block(logging_block);

        frame.render_widget(logging_view, logging);
    }

    fn run(mut self, mut terminal: DefaultTerminal) -> Result<()> {
        loop {
            if let Some(input) = self.rtt.up_channel(0) {
                let mut buf = [0u8; 1024];
                let mut core = self.session.core(0)?;
                let count = input.read(&mut core, &mut buf)?;

                self.defmt_buffer.extend_from_slice(&buf[..count]);

                let mut defmt_decoder = self.defmt_table.new_stream_decoder();
                defmt_decoder.received(&self.defmt_buffer);

                loop {
                    match defmt_decoder.decode() {
                        Ok(frame) => {
                            let mut line = Line::default();

                            if let Some(timestamp) = frame.display_timestamp() {
                                let span =
                                    Span::raw(timestamp.to_string()).style(Style::new().gray());
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

                            self.logging_lines.push(line);
                        }
                        Err(DecodeError::Malformed)
                            if self.defmt_table.encoding().can_recover() =>
                        {
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
            }

            terminal.draw(|frame| self.draw(frame))?;

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
        Ok(())
    }
}
