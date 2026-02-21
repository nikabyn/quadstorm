use std::fmt::Display;

use ratatui::{
    Frame,
    layout::Rect,
    text::Line,
    widgets::{Block, Paragraph},
};

pub struct LogsTab {
    pub kind: LogsTabKind,
    pub lines: Vec<Line<'static>>,
}

impl LogsTab {
    pub fn new(kind: LogsTabKind) -> Self {
        Self {
            kind,
            lines: Vec::new(),
        }
    }

    pub fn draw(&self, frame: &mut Frame, area: Rect) {
        let log_block = Block::bordered().title(self.kind.to_string());
        let num_lines = log_block.inner(area).height;
        let log_lines: Vec<_> = self
            .lines
            .iter()
            .rev()
            .take(num_lines as usize)
            .rev()
            .cloned()
            .collect();
        let logging_view = Paragraph::new(log_lines).block(log_block);

        frame.render_widget(logging_view, area);
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LogsTabKind {
    Remote = 0,
    Relay = 1,
    Drone = 2,
}

impl Display for LogsTabKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Self::Remote => "Remote",
            Self::Relay => "Relay",
            Self::Drone => "Drone",
        })
    }
}
