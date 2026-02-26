use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::ecs::system::Res;
use bevy::ecs::{prelude::Result as BevyResult, system::Local};
use bevy::log::Level;
use bevy::time::Time;
use bevy_egui::EguiContexts;
use bevy_egui::egui::{self, Button, Color32, RichText, ScrollArea, Ui};
use common_messages::{DroneResponse, RemoteRequest};
use egui_plot::PlotPoint;

use crate::PingStatus;
use crate::rtt::{DroneMessage, LogMessage, LogSource, RemoteMessage};

pub fn ui_system(
    // External state
    time: Res<Time>,
    mut contexts: EguiContexts,
    ping_status: Res<PingStatus>,

    // Internal state
    mut active_tab: Local<usize>,
    mut settings: Local<Settings>,
    mut telemetry: Local<CollectedTelemetry>,
    mut relay_logs: Local<Vec<(bevy::log::Level, String)>>,
    mut drone_logs: Local<Vec<(bevy::log::Level, String)>>,

    // Messages
    mut drone_msgs: MessageReader<DroneMessage>,
    remote_msgs: MessageWriter<RemoteMessage>,
    mut log_msgs: MessageReader<LogMessage>,
) -> BevyResult {
    for DroneMessage(drone_res) in drone_msgs.read() {
        if let &DroneResponse::Telemetry(sample) = drone_res {
            let t = time.elapsed().as_millis() as f64;

            for i in 0..3 {
                telemetry.orientation[i].push(PlotPoint::new(t, sample.orientation[i] as f64));
            }
            telemetry
                .thrust
                .push(PlotPoint::new(t, sample.thrust as f64));
            telemetry.armed.push(PlotPoint::new(
                t,
                sample.armed.then_some(1.0).unwrap_or(0.0),
            ));
            for i in 0..3 {
                telemetry.output[i].push(PlotPoint::new(t, sample.output[i] as f64));
            }
            for i in 0..4 {
                telemetry.throttles[i].push(PlotPoint::new(t, sample.throttles[i] as f64));
            }
        }
    }
    for LogMessage(src, level, message) in log_msgs.read() {
        match src {
            LogSource::Relay => relay_logs.push((*level, message.to_owned())),
            LogSource::Drone => drone_logs.push((*level, message.to_owned())),
        }
    }

    let ctx = contexts.ctx_mut()?;

    egui::TopBottomPanel::new(egui::panel::TopBottomSide::Bottom, "panel_bottom")
        .show_separator_line(true)
        .show(ctx, |ui| draw_statusbar(ui, &ping_status));

    egui::SidePanel::new(egui::panel::Side::Right, "panel_right")
        .resizable(false)
        .exact_width(300.0)
        .show(ctx, |ui| {
            ui.take_available_width();
            draw_settings(ui, &mut settings, remote_msgs);
        });

    egui::TopBottomPanel::new(egui::panel::TopBottomSide::Top, "panel_top")
        .show_separator_line(false)
        .show(ctx, |ui| draw_navbar(ui, &mut active_tab));

    egui::CentralPanel::default().show(ctx, |ui| match *active_tab {
        0 => draw_telemetry(ui, &telemetry),
        1 => {}
        2 => draw_logs(ui, &relay_logs),
        3 => draw_logs(ui, &drone_logs),
        _ => {}
    });

    Ok(())
}

pub fn draw_navbar(ui: &mut Ui, active_tab: &mut usize) {
    ui.add_space(8.);
    ui.horizontal(|ui| {
        let clicked = ["Telemetry", "Preview", "Relay Logs", "Drone Logs"]
            .iter()
            .enumerate()
            .map(|(i, &label)| {
                let button = if *active_tab == i {
                    Button::new(RichText::new(label).color(Color32::WHITE))
                        .fill(Color32::from_rgb(86, 79, 173))
                } else {
                    Button::new(label)
                }
                .min_size(egui::Vec2::new(0., 24.));
                (i, ui.add(button))
            })
            .map(|(i, response)| response.clicked().then_some(i))
            .reduce(Option::or)
            .flatten();

        if let Some(tab_index) = clicked {
            *active_tab = tab_index;
        }
    });
}

pub fn draw_statusbar(ui: &mut Ui, ping_status: &PingStatus) {
    ui.horizontal(|ui| {
        ui.label("Drone: ");
        if let Some(rtt) = ping_status.roundtrip_drone {
            ui.label(
                RichText::new(format!("Connected, rtt={}ms", rtt.as_millis()))
                    .color(Color32::LIGHT_GREEN),
            );
        } else {
            ui.label(RichText::new("Not connected").color(Color32::LIGHT_RED));
        }

        ui.add_space(8.0);

        ui.label("Relay: ");
        if let Some(rtt) = ping_status.roundtrip_relay {
            ui.label(
                RichText::new(format!("Connected, rtt={}ms", rtt.as_millis()))
                    .color(Color32::LIGHT_GREEN),
            );
        } else {
            ui.label(RichText::new("Not connected").color(Color32::LIGHT_RED));
        }

        ui.add_space(8.0);

        ui.label("Controller: ");
        ui.label(RichText::new("Not connected").color(Color32::LIGHT_RED));
    });
}

#[derive(Default)]
pub struct CollectedTelemetry {
    orientation: [Vec<PlotPoint>; 3],
    thrust: Vec<PlotPoint>,
    armed: Vec<PlotPoint>,
    output: [Vec<PlotPoint>; 3],
    throttles: [Vec<PlotPoint>; 4],
}

pub fn draw_telemetry(ui: &mut Ui, telemetry: &CollectedTelemetry) {
    let legend = egui_plot::Legend::default()
        .follow_insertion_order(true)
        .grouping(egui_plot::LegendGrouping::ById)
        .position(egui_plot::Corner::LeftTop);
    let mapped = [
        ("orientation x", &telemetry.orientation[0]),
        ("orientation y", &telemetry.orientation[1]),
        ("orientation z", &telemetry.orientation[2]),
        ("armed", &telemetry.armed),
        ("thrust", &telemetry.thrust),
        ("output yaw", &telemetry.output[0]),
        ("output pitch", &telemetry.output[1]),
        ("output roll", &telemetry.output[2]),
        ("throttle 0", &telemetry.throttles[0]),
        ("throttle 1", &telemetry.throttles[1]),
        ("throttle 2", &telemetry.throttles[2]),
        ("throttle 3", &telemetry.throttles[3]),
    ];

    ui.with_layout(egui::Layout::left_to_right(egui::Align::Min), |ui| {
        egui_plot::Plot::new("input_plot")
            .legend(legend.clone())
            .width(ui.available_width() / 2.)
            .show(ui, |plot_ui| {
                for (label, data) in &mapped[..5] {
                    plot_ui.line(egui_plot::Line::new(
                        *label,
                        egui_plot::PlotPoints::Borrowed(data),
                    ));
                }
            });

        egui_plot::Plot::new("output_plot")
            .legend(legend)
            .show(ui, |plot_ui| {
                for (label, data) in &mapped[5..] {
                    plot_ui.line(egui_plot::Line::new(
                        *label,
                        egui_plot::PlotPoints::Borrowed(data),
                    ));
                }
            });
    });
}

fn draw_logs(ui: &mut Ui, logs: &[(Level, String)]) {
    ScrollArea::both()
        .animated(true)
        .stick_to_bottom(true)
        .show(ui, |ui| {
            ui.take_available_space();
            for (level, message) in logs {
                let color = match *level {
                    Level::TRACE => Color32::WHITE,
                    Level::DEBUG => Color32::LIGHT_BLUE,
                    Level::INFO => Color32::LIGHT_GREEN,
                    Level::WARN => Color32::LIGHT_YELLOW,
                    Level::ERROR => Color32::LIGHT_RED,
                };
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(level.as_str().to_lowercase())
                            .color(color)
                            .monospace(),
                    );
                    ui.label(RichText::new(message).monospace());
                });
            }
        });
}

#[derive(Default)]
pub struct Settings {
    tune: [f32; 3],
}

pub fn draw_settings(
    ui: &mut Ui,
    settings: &mut Settings,
    mut remote_msgs: MessageWriter<RemoteMessage>,
) {
    ui.add_space(8.);
    ui.label(RichText::new("Settings").size(16.0).strong());

    ui.add_space(16.);

    ui.label(RichText::new("Arming").strong());
    let arm_button = ui.add_sized([ui.available_width(), 0.0], Button::new("Send arm"));
    if arm_button.clicked() {
        remote_msgs.write(RemoteMessage(RemoteRequest::SetArm(true)));
    }
    let disarm_button = ui.add_sized([ui.available_width(), 0.0], Button::new("Send disarm"));
    if disarm_button.clicked() {
        remote_msgs.write(RemoteMessage(RemoteRequest::SetArm(false)));
    }

    ui.add_space(16.);

    ui.label(RichText::new("Tune").strong());
    ui.columns(3, |cols| {
        for (i, col) in cols.iter_mut().enumerate() {
            col.add(egui::DragValue::new(&mut settings.tune[i]).max_decimals(4));
        }
    });
    let update_button = ui.add_sized([ui.available_width(), 0.0], Button::new("Send"));
    if update_button.clicked() {
        remote_msgs.write(RemoteMessage(RemoteRequest::SetTune {
            kp: settings.tune,
            ki: settings.tune,
            kd: settings.tune,
        }));
    }

    ui.add_space(16.);

    ui.label(RichText::new("Reset").strong());
    let reset_button = ui.add_sized([ui.available_width(), 0.0], Button::new("Send"));
    if reset_button.clicked() {
        remote_msgs.write(RemoteMessage(RemoteRequest::Reset));
    }
}
