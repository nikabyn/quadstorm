use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::mpsc::TryRecvError;
use std::thread;
use std::time::Duration;

use anyhow::{Result, anyhow};
use common_messages::{DroneResponse, RemoteRequest};
use ratatui::DefaultTerminal;
use ratatui::crossterm::event::{self, Event, KeyCode};
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::{Color, Style, Styled};
use ratatui::text::Line;

mod rtt;

mod control_tab;

mod logs_tab;
use logs_tab::{LogsTab, LogsTabKind};

use crate::control_tab::ControlTab;
use crate::rtt::rtt_communicate;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let Some(relay_elf_path) = args.next() else {
        return Err(anyhow!("Expected path to relay elf as first argument"));
    };
    let Some(drone_elf_path) = args.next() else {
        return Err(anyhow!("Expected path to drone elf as second argument"));
    };
    let relay_elf = std::fs::read(relay_elf_path)?;
    let drone_elf = std::fs::read(drone_elf_path)?;

    let terminal = ratatui::init();
    let app_result = App::new().start(terminal, relay_elf, drone_elf);
    ratatui::restore();
    app_result
}

struct App {
    control_tab: ControlTab,
    active_tab: usize,
    // tab index 0, 1, 2
    logs_tabs: [LogsTab; 3],
}

impl App {
    fn new() -> Self {
        Self {
            active_tab: LogsTabKind::Remote as usize,
            logs_tabs: [
                LogsTab::new(LogsTabKind::Remote),
                LogsTab::new(LogsTabKind::Relay),
                LogsTab::new(LogsTabKind::Drone),
            ],
            control_tab: ControlTab::new(),
        }
    }

    fn draw(&self, frame: &mut ratatui::Frame) {
        let layout = Layout::new(
            Direction::Horizontal,
            [Constraint::Fill(1), Constraint::Fill(2)],
        );
        let [controls, logging] = layout.areas(frame.area());
        self.control_tab.draw(frame, controls);
        self.logs_tabs[self.active_tab].draw(frame, logging);
    }

    fn start(
        self,
        terminal: DefaultTerminal,
        relay_elf: Vec<u8>,
        drone_elf: Vec<u8>,
    ) -> Result<()> {
        let (tx_logs, rx_logs) = mpsc::channel::<(LogsTabKind, Line)>();
        let (tx_drone_res, rx_drone_res) = mpsc::channel::<DroneResponse>();
        let (tx_remote_req, rx_remote_req) = mpsc::channel::<RemoteRequest>();

        let tui_closed = AtomicBool::new(false);

        thread::scope(|s| {
            s.spawn(|| {
                let rx_remote_req = rx_remote_req;
                let tx_drone_res = tx_drone_res;
                let tx_logs = tx_logs;

                while !tui_closed.load(Ordering::Relaxed) {
                    let result = rtt_communicate(
                        &relay_elf,
                        &drone_elf,
                        &rx_remote_req,
                        &tx_drone_res,
                        &tx_logs,
                    );
                    let message = match result {
                        Ok(..) => {
                            Line::from_iter(["Thread terminated while communicating with relay."
                                .set_style(Color::Yellow)])
                        }
                        Err(err) => Line::from_iter([
                            "[".set_style(Style::default()),
                            "Error".set_style(Color::Red),
                            "] ".set_style(Style::default()),
                            err.to_string().set_style(Style::default()),
                        ]),
                    };
                    _ = tx_logs.send((LogsTabKind::Remote, Line::from_iter(message)));
                    thread::sleep(Duration::from_millis(100));
                }
            });
            let result = self.run(terminal, rx_drone_res, tx_remote_req, rx_logs);
            tui_closed.store(true, Ordering::Relaxed);
            result
        })
    }

    fn run(
        mut self,
        mut terminal: DefaultTerminal,
        drone_res: mpsc::Receiver<DroneResponse>,
        remote_req: mpsc::Sender<RemoteRequest>,
        logs: mpsc::Receiver<(LogsTabKind, Line<'static>)>,
    ) -> Result<()> {
        let tick_rate = Duration::from_millis(5);

        loop {
            match logs.try_recv() {
                Ok((tab, line)) => self.logs_tabs[tab as usize].lines.push(line),
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }
            match drone_res.try_recv() {
                Ok(res) => self.logs_tabs[LogsTabKind::Remote as usize]
                    .lines
                    .push(Line::from(format!("Received: {res:?}"))),
                Err(TryRecvError::Disconnected) => break,
                Err(TryRecvError::Empty) => {}
            }

            terminal.draw(|frame| self.draw(frame))?;

            if event::poll(tick_rate)? {
                let event = event::read()?;

                if self.control_tab.handle_event(&event, remote_req.clone()) {
                    continue;
                }

                if let Event::Key(key) = event {
                    self.logs_tabs[LogsTabKind::Remote as usize]
                        .lines
                        .push(Line::from(format!("Pressed <{}>", key.code.to_string())));
                    match key.code {
                        KeyCode::Esc | KeyCode::Char('q') => break,
                        KeyCode::Char('1') => self.active_tab = 0,
                        KeyCode::Char('2') => self.active_tab = 1,
                        KeyCode::Char('3') => self.active_tab = 2,
                        KeyCode::Char('i') => self.control_tab.toggle_input(),
                        KeyCode::Char('p') => remote_req.send(RemoteRequest::Ping)?,
                        _ => {}
                    }
                }
            }
        }

        Ok(())
    }
}
