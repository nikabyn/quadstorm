use std::{collections::HashMap, iter::Peekable, sync::mpsc::Sender};

use anyhow::{Result, anyhow, bail};
use common_messages::RemoteRequest;
use logos::{Lexer, Logos};
use ratatui::{
    crossterm::event::{Event, KeyCode},
    layout::{Constraint, Layout, Rect},
    style::{Color, Style, Styled},
    widgets::{Block, Borders, Paragraph},
};
use tui_input::{Input, backend::crossterm::EventHandler};

#[derive(PartialEq, Eq)]
pub enum InputMode {
    Inactive,
    Editing,
}

pub struct ControlTab {
    input_mode: InputMode,
    input: Input,
    parsed_input: Result<Option<RemoteRequest>>,
}

impl ControlTab {
    pub fn new() -> Self {
        Self {
            input_mode: InputMode::Inactive,
            input: Input::default(),
            parsed_input: Ok(None),
        }
    }

    pub fn handle_event(&mut self, event: &Event, tx_remote_req: Sender<RemoteRequest>) -> bool {
        if self.input_mode == InputMode::Inactive {
            return false;
        }

        if let Some(key) = event.as_key_event() {
            match key.code {
                KeyCode::Esc => self.input_mode = InputMode::Inactive,
                KeyCode::Enter => {
                    let mut parsed = Ok(None);
                    core::mem::swap(&mut parsed, &mut self.parsed_input);
                    if let Ok(Some(req)) = parsed {
                        _ = tx_remote_req.send(req);
                    }
                    self.input.reset();
                    self.input_mode = InputMode::Inactive;
                }
                _ => {}
            }
        }

        self.input.handle_event(event);
        self.parsed_input = parse_input(self.input.value());

        true
    }

    pub fn draw(&self, frame: &mut ratatui::Frame, area: Rect) {
        let vertical = Layout::vertical([
            Constraint::Length(1),
            Constraint::Length(3),
            Constraint::Fill(1),
        ]);
        let [title_area, input_area, feedback_area] = vertical.areas(area);

        let title_style = Style::from(Color::White);
        let mut title = Block::bordered()
            .borders(Borders::TOP)
            .style(title_style)
            .title("Control");

        // keep 2 for borders and 1 for cursor
        let width = input_area.width.max(3) - 3;
        let scroll = self.input.visual_scroll(width as usize);
        let text_style = Color::Gray;
        let input_style = Style::from(match self.parsed_input {
            Ok(None) => Color::White,
            Ok(Some(_)) => Color::LightGreen,
            Err(_) => Color::LightRed,
        });
        let mut input = Paragraph::new(self.input.value())
            .style(text_style)
            .scroll((0, scroll as u16))
            .block(Block::bordered().border_style(input_style).title("Input"));

        if self.input_mode == InputMode::Editing {
            // Ratatui hides the cursor unless it's explicitly set. Position the  cursor past the
            // end of the input text and one line down from the border to the input line
            let x = self.input.visual_cursor().max(scroll) - scroll + 1;
            frame.set_cursor_position((input_area.x + x as u16, input_area.y + 1))
        }

        let feedback = Paragraph::new(format!("{:?}", self.parsed_input));

        if !self.is_active() {
            title = title.set_style(title_style.dim());
            input = input.set_style(input_style.dim());
        }
        frame.render_widget(title, title_area);
        frame.render_widget(input, input_area);
        frame.render_widget(feedback, feedback_area);
    }

    pub fn toggle_input(&mut self) {
        self.input_mode = if self.input_mode == InputMode::Editing {
            InputMode::Inactive
        } else {
            InputMode::Editing
        };
    }

    pub fn is_active(&self) -> bool {
        self.input_mode == InputMode::Editing
    }
}

/// Format
/// ```
/// Ping
/// SetArm(bool)
/// ArmConfirm
/// SetThrust(f32)
/// SetTarget([f32, f32, f32])
/// SetTune(kp:[f32, f32, f32],ki:[f32, f32, f32],kd:[f32, f32, f32])
/// ```
fn parse_input(text: &str) -> Result<Option<RemoteRequest>> {
    if text.trim().is_empty() {
        return Ok(None);
    }
    let mut tokens = Token::lexer(text).peekable();

    fn consume(tokens: &mut Peekable<Lexer<'_, Token>>, token: Token) -> Result<()> {
        let Some(result) = tokens.next() else {
            bail!("Expected {token:?} got nothing");
        };
        match result {
            Ok(tok) if tok == token => Ok(()),
            Ok(tok) => bail!("Expected {token:?} got {tok:?}"),
            Err(_) => bail!("Expected {token:?} got invalid token"),
        }
    }

    fn consume_or_not(tokens: &mut Peekable<Lexer<'_, Token>>, token: Token) {
        if tokens.peek() == Some(&Ok(token)) {
            _ = tokens.next();
        }
    }

    fn consume_float(tokens: &mut Peekable<Lexer<'_, Token>>) -> Result<f32> {
        let Some(Ok(Token::Float(float))) = tokens.next() else {
            bail!("Expected float");
        };
        Ok(float)
    }

    fn consume_ident(tokens: &mut Peekable<Lexer<'_, Token>>) -> Result<String> {
        let Some(Ok(Token::Ident(ident))) = tokens.next() else {
            bail!("Expected ident");
        };
        Ok(ident)
    }

    let Some(Ok(Token::Ident(variant))) = tokens.next() else {
        bail!("Expected message variant");
    };

    Ok(Some(match variant.as_str() {
        "Ping" => RemoteRequest::Ping,
        "SetArm" => {
            consume(&mut tokens, Token::ParenOpen)?;
            let Some(Ok(Token::Bool(value))) = tokens.next() else {
                bail!("Expected bool");
            };
            consume(&mut tokens, Token::ParenClose)?;
            RemoteRequest::SetArm(value)
        }
        "ArmConfirm" => RemoteRequest::ArmConfirm,
        "SetThrust" => {
            consume(&mut tokens, Token::ParenOpen)?;
            let value = consume_float(&mut tokens)?;
            consume(&mut tokens, Token::ParenClose)?;
            RemoteRequest::SetThrust(value)
        }
        "SetTarget" => {
            let mut values = [0.0; 3];

            consume(&mut tokens, Token::ParenOpen)?;
            consume(&mut tokens, Token::BracketOpen)?;
            for value in &mut values {
                *value = consume_float(&mut tokens)?;
                consume_or_not(&mut tokens, Token::Comma);
            }
            consume(&mut tokens, Token::BracketClose)?;
            consume(&mut tokens, Token::ParenClose)?;

            RemoteRequest::SetTarget(values)
        }
        "SetTune" => {
            let mut values = HashMap::new();

            consume(&mut tokens, Token::ParenOpen)?;
            for _ in 0..3 {
                let key = consume_ident(&mut tokens)?;
                consume_or_not(&mut tokens, Token::Colon);

                let mut value = [0.0; 3];
                consume(&mut tokens, Token::BracketOpen)?;
                for float in &mut value {
                    *float = consume_float(&mut tokens)?;
                    consume_or_not(&mut tokens, Token::Comma);
                }
                consume(&mut tokens, Token::BracketClose)?;
                consume_or_not(&mut tokens, Token::Comma);

                values.insert(key, value);
            }
            consume(&mut tokens, Token::ParenClose)?;

            RemoteRequest::SetTune {
                kp: *values.get("kp").ok_or(anyhow!("Missing key kp"))?,
                ki: *values.get("ki").ok_or(anyhow!("Missing key ki"))?,
                kd: *values.get("kd").ok_or(anyhow!("Missing key kd"))?,
            }
        }
        _ => bail!("Invalid message variant: {variant}"),
    }))
}

fn parse_float(text: &str) -> Option<f32> {
    let text: String = text.chars().filter(|c| *c != '_').collect();
    text.parse().ok()
}

#[derive(Logos, PartialEq, Debug)]
#[logos(skip r"[ \t\n]+")]
enum Token {
    #[regex("[a-zA-Z_]+[a-zA-Z_0-9]*", |lex| lex.slice().to_owned())]
    Ident(String),

    #[regex(r"[0-9][0-9_]*\.[0-9_]+", |lex| parse_float(lex.slice()))]
    #[regex(r"[0-9][0-9_]*(\.)?", |lex| parse_float(lex.slice()))]
    #[regex(r"\.[0-9_]+", |lex| parse_float(lex.slice()))]
    Float(f32),
    #[regex("true", |_| true)]
    #[regex("false", |_| false)]
    Bool(bool),

    #[token(",")]
    Comma,
    #[token(":")]
    Colon,

    #[token("(")]
    ParenOpen,
    #[token(")")]
    ParenClose,
    #[token("[")]
    BracketOpen,
    #[token("]")]
    BracketClose,
}
