use std::time::Duration;

use anyhow::{Result as AnyResult, anyhow};
use bevy::DefaultPlugins;
use bevy::app::{App, AppExit, FixedPostUpdate, FixedUpdate, Update};
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::ecs::name::Name;
use bevy::ecs::query::Changed;
use bevy::ecs::resource::Resource;
use bevy::ecs::system::{IntoSystem, Local, Query, Res, ResMut};
use bevy::input::ButtonState;
use bevy::input::gamepad::{Gamepad, GamepadAxis, GamepadButton};
use bevy::input::keyboard::{KeyCode, KeyboardInput};
use bevy::input_focus::InputDispatchPlugin;
use bevy::input_focus::tab_navigation::TabNavigationPlugin;
use bevy::log::{Level, debug, error, error_once, info, trace, warn};
use bevy::time::Time;
use common_messages::RemoteRequest;

mod rtt;
use rtt::{
    DroneMessage, DroneTag, ElfResource, LogMessage, RelayTag, RemoteMessage,
    rtt_communication_system,
};

use crate::rtt::{LogSource, log_error_system};

fn main() -> AnyResult<()> {
    let mut args = std::env::args().skip(1);
    let Some(relay_elf_path) = args.next() else {
        return Err(anyhow!("Expected path to relay elf as first argument"));
    };
    let Some(drone_elf_path) = args.next() else {
        return Err(anyhow!("Expected path to drone elf as second argument"));
    };

    App::new()
        .add_plugins((DefaultPlugins, InputDispatchPlugin, TabNavigationPlugin))
        .insert_resource(KeepArmed(false))
        .insert_resource(ElfResource::<RelayTag>::new(relay_elf_path)?)
        .insert_resource(ElfResource::<DroneTag>::new(drone_elf_path)?)
        .add_message::<RemoteMessage>()
        .add_message::<DroneMessage>()
        .add_message::<LogMessage>()
        .add_systems(Update, (keyboard_input_system, gamepad_input_system))
        .add_systems(FixedUpdate, rtt_communication_system.pipe(log_error_system))
        .add_systems(FixedUpdate, keep_armed_system)
        .add_systems(FixedPostUpdate, log_logs)
        .run();

    Ok(())
}

fn log_logs(mut logs: MessageReader<LogMessage>) {
    for LogMessage(source, level, message) in logs.read() {
        if *source == LogSource::Relay {
            continue;
        }
        match (*level, *source) {
            (Level::ERROR, LogSource::Drone) => error!(target: "drone", "{message}"),
            (Level::WARN, LogSource::Drone) => warn!(target: "drone", "{message}"),
            (Level::INFO, LogSource::Drone) => info!(target: "drone", "{message}"),
            (Level::DEBUG, LogSource::Drone) => debug!(target: "drone", "{message}"),
            (Level::TRACE, LogSource::Drone) => trace!(target: "drone", "{message}"),
            (Level::ERROR, LogSource::Relay) => error!(target: "relay", "{message}"),
            (Level::WARN, LogSource::Relay) => warn!(target: "relay", "{message}"),
            (Level::INFO, LogSource::Relay) => info!(target: "relay", "{message}"),
            (Level::DEBUG, LogSource::Relay) => debug!(target: "relay", "{message}"),
            (Level::TRACE, LogSource::Relay) => trace!(target: "relay", "{message}"),
        };
    }
}

#[derive(Resource)]
struct KeepArmed(bool);

fn keep_armed_system(
    keep_armed: Res<KeepArmed>,
    time: Res<Time>,
    mut time_last_sent: Local<Duration>,
    mut remote_msgs: MessageWriter<RemoteMessage>,
) {
    let current = time.elapsed();
    let time_has_elapsed = (current - *time_last_sent) >= Duration::from_millis(100);
    if keep_armed.0 && time_has_elapsed {
        *time_last_sent = current;
        remote_msgs.write(RemoteMessage(RemoteRequest::ArmConfirm));
    }
}

fn keyboard_input_system(
    mut inputs: MessageReader<KeyboardInput>,
    mut remote_msgs: MessageWriter<RemoteMessage>,
    mut keep_armed: ResMut<KeepArmed>,
    mut exit: MessageWriter<AppExit>,
) {
    for input in inputs.read().filter(|i| i.state == ButtonState::Released) {
        match input.key_code {
            KeyCode::Escape | KeyCode::KeyQ => {
                exit.write(AppExit::Success);
            }
            KeyCode::KeyA => {
                keep_armed.0 = true;
                remote_msgs.write(RemoteMessage(RemoteRequest::SetArm(true)));
            }
            KeyCode::KeyD => {
                keep_armed.0 = false;
                remote_msgs.write(RemoteMessage(RemoteRequest::SetArm(false)));
            }
            KeyCode::KeyP => {
                remote_msgs.write(RemoteMessage(RemoteRequest::Ping));
            }
            _ => {}
        };
    }
}

fn gamepad_input_system(
    gamepads: Query<(&Name, &Gamepad), Changed<Gamepad>>,
    mut remote_msgs: MessageWriter<RemoteMessage>,
    mut keep_armed: ResMut<KeepArmed>,
) {
    let Some((_, gamepad)) = gamepads.iter().next() else {
        error_once!("No gamepad connected.");
        return;
    };

    let pitch = gamepad.get(GamepadAxis::LeftStickY).unwrap();
    let roll = gamepad.get(GamepadAxis::LeftStickX).unwrap();
    let yaw = gamepad.get(GamepadAxis::RightStickX).unwrap();
    let thrust = gamepad.get(GamepadAxis::LeftZ).unwrap();

    let pitch = pitch * 30.0;
    let roll = roll * 30.0;
    let yaw = yaw * 30.0;
    let thrust = ((thrust / 2.0) + 0.5) * 1000.0;

    debug!(
        "pitch={}, roll={}, yaw={}, thrust={}",
        pitch, roll, yaw, thrust
    );
    remote_msgs.write_batch([
        RemoteMessage(RemoteRequest::SetTarget([roll, pitch, yaw])),
        RemoteMessage(RemoteRequest::SetThrust(thrust)),
    ]);

    let armed_left = gamepad
        .get(GamepadButton::North)
        .map(|v| v == 1.0)
        .unwrap_or(false);
    let armed_right = gamepad
        .get(GamepadButton::East)
        .map(|v| v == 1.0)
        .unwrap_or(false);
    let armed = armed_left && armed_right;
    if keep_armed.0 != armed {
        info!("armed: {armed}");
        remote_msgs.write(RemoteMessage(RemoteRequest::SetArm(armed)));
        keep_armed.0 = armed;
    }
}
