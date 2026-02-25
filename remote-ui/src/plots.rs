use bevy::ecs::{query::QuerySingleError, resource::Resource};
use bevy::prelude::Result;
use bevy_egui::{EguiContexts, egui};

#[derive(Debug, Default, Resource)]
pub struct PlotData {}

pub fn draw_telemtry(contexts: &mut EguiContexts) -> Result {
    egui::Window::new("Hello").show(contexts.ctx_mut()?, |ui| {
        ui.label("world");
    });

    Ok(())
}
