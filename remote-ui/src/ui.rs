use bevy::ecs::{
    component::Component,
    message::{MessageReader, MessageWriter},
    prelude::Result as BevyResult,
    system::{Commands, Local, Query},
};
use bevy::log::info;
use bevy_egui::{
    EguiContexts,
    egui::{self, Color32, Ui, Vec2b, vec2},
};
use common_messages::{DroneResponse, RemoteRequest, Telemetry};
use egui_plot::PlotPoint;

use crate::rtt::{DroneMessage, RemoteMessage};

#[derive(Default)]
pub struct CollectedTelemetry {
    orientation: [Vec<PlotPoint>; 3],
    thrust: Vec<PlotPoint>,
    armed: Vec<PlotPoint>,
    output: [Vec<PlotPoint>; 3],
    throttles: [Vec<PlotPoint>; 4],
}

#[derive(Default)]
pub struct Settings {
    kp: [f32; 3],
    ki: [f32; 3],
    kd: [f32; 3],
}

pub fn ui_system(
    mut contexts: EguiContexts,
    mut active_tab: Local<usize>,
    mut settings: Local<Settings>,
    mut telemetry: Local<CollectedTelemetry>,
    mut drone_msgs: MessageReader<DroneMessage>,
    mut remote_msgs: MessageWriter<RemoteMessage>,
) -> BevyResult {
    for DroneMessage(drone_res) in drone_msgs.read() {
        if let &DroneResponse::Telemetry(sample) = drone_res {
            info!("{sample}");
            let t = sample.timestamp as f64;

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

    let ctx = contexts.ctx_mut()?;

    egui::CentralPanel::default().show(ctx, |ui| {
        draw_ui(ui, active_tab, settings, telemetry, drone_msgs);
    });

    Ok(())
}

pub fn draw_ui(
    ui: &mut Ui,
    mut active_tab: Local<usize>,
    mut settings: Local<Settings>,
    mut telemetry: Local<CollectedTelemetry>,
    mut drone_msgs: MessageReader<DroneMessage>,
) {
    ui.horizontal(|ui| {
        let clicked = ["Telemetry", "Preview", "Relay Logs", "Drone Logs"]
            .iter()
            .enumerate()
            .map(|(i, &label)| {
                let button = egui::Button::new(label).min_size(egui::Vec2::new(0., 24.));
                let button = if *active_tab == i {
                    button.fill(Color32::PURPLE)
                } else {
                    button
                };
                (i, ui.add(button))
            })
            .map(|(i, response)| response.clicked().then_some(i))
            .reduce(Option::or)
            .flatten();

        if let Some(tab_index) = clicked {
            *active_tab = tab_index;
        }
    });

    ui.add_space(10.);

    if *active_tab == 0 {
        draw_telemetry(ui, &telemetry);
    }

    // TODO draw settings

    // ui.label("Tune");

    // // ui.label("ki");
    // ui.horizontal(|ui| {
    //     for i in 0..3 {
    //         ui.add(egui::DragValue::new(&mut settings.kp[i]).max_decimals(4));
    //     }
    // });
    // // ui.label("ki");
    // // ui.horizontal(|ui| {
    // //     for i in 0..3 {
    // //         ui.add(egui::DragValue::new(&mut ui_state.ki[i]));
    // //     }
    // // });
    // // ui.label("kd");
    // // ui.horizontal(|ui| {
    // //     for i in 0..3 {
    // //         ui.add(egui::DragValue::new(&mut ui_state.kd[i]));
    // //     }
    // // });

    // if ui.button("Send tune").clicked() {
    //     remote_msgs.write(RemoteMessage(RemoteRequest::SetTune {
    //         kp: settings.kp,
    //         ki: settings.kp,
    //         kd: settings.kp,
    //     }));
    // }
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
