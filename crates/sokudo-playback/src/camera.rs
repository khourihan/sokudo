use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::{input::mouse::MouseMotion, prelude::*};

pub struct PanOrbitPlugin;

impl Plugin for PanOrbitPlugin {
    fn build(&self, app: &mut App) {
        app
            .add_systems(Startup, setup_pan_orbit_camera)
            .add_systems(Update, pan_orbit_camera.run_if(any_with_component::<PanOrbitState>));
    }
}

#[derive(Bundle, Default)]
pub struct PanOrbitCameraBundle {
    pub camera: Camera3dBundle,
    pub state: PanOrbitState,
    pub settings: PanOrbitSettings,
}

#[derive(Component)]
pub struct PanOrbitState {
    pub center: Vec3,
    pub radius: f32,
    pub upside_down: bool,
    pub pitch: f32,
    pub yaw: f32,
}

#[derive(Component)]
pub struct PanOrbitSettings {
    pub pan_sensitivity: f32,
    pub orbit_sensitivity: f32,
    pub zoom_sensitivity: f32,
    pub pan_key: Option<KeyCode>,
    pub orbit_key: Option<KeyCode>,
    pub zoom_key: Option<KeyCode>,
}

impl Default for PanOrbitState {
    fn default() -> Self {
        PanOrbitState {
            center: Vec3::ZERO,
            radius: 1.0,
            upside_down: false,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

impl Default for PanOrbitSettings {
    fn default() -> Self {
        PanOrbitSettings {
            pan_sensitivity: 0.001, // 1000 pixels per world unit
            orbit_sensitivity: 0.1f32.to_radians(), // 0.1 degree per pixel
            zoom_sensitivity: 0.01,
            pan_key: Some(KeyCode::ShiftLeft),
            orbit_key: None,
            zoom_key: Some(KeyCode::ControlLeft),
        }
    }
}

fn setup_pan_orbit_camera(
    mut commands: Commands,
) {
    commands.spawn(PanOrbitCameraBundle::default());
}

fn pan_orbit_camera(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut evr_motion: EventReader<MouseMotion>,
    mut cameras: Query<(&PanOrbitSettings, &mut PanOrbitState, &mut Transform)>,
) {
    let mut total_motion: Vec2 = evr_motion.read()
        .map(|ev| ev.delta).sum();

    total_motion.y = -total_motion.y;

    let left_pressed = mouse.pressed(MouseButton::Left);

    for (settings, mut state, mut transform) in &mut cameras {
        let is_panning = settings.pan_key.map(|key| keys.pressed(key)).unwrap_or(true);
        let is_orbiting = settings.orbit_key.map(|key| keys.pressed(key)).unwrap_or(true);
        let is_zooming = settings.zoom_key.map(|key| keys.pressed(key)).unwrap_or(true);

        let mut total_pan = Vec2::ZERO;
        if is_panning && left_pressed {
            total_pan -= total_motion * settings.pan_sensitivity;
        }

        let mut total_orbit = Vec2::ZERO;
        if is_orbiting && left_pressed && !is_panning && !is_zooming {
            total_orbit -= total_motion * settings.orbit_sensitivity * Vec2::new(1.0, -1.0);
        }

        let mut total_zoom = Vec2::ZERO;
        if is_zooming && left_pressed {
            total_zoom -= total_motion * settings.zoom_sensitivity;
        }

        if mouse.just_pressed(MouseButton::Left) {
            state.upside_down = state.pitch < -FRAC_PI_2 || state.pitch > FRAC_PI_2;
        }

        if state.upside_down {
            total_orbit.x = -total_orbit.x;
        }

        let mut any = false;

        if total_zoom != Vec2::ZERO {
            any = true;
            state.radius *= (-total_zoom.y).exp();
        }

        if total_orbit != Vec2::ZERO {
            any = true;
            state.yaw += total_orbit.x;
            state.pitch += total_orbit.y;

            if state.yaw > PI {
                state.yaw -= TAU;
            }
            if state.yaw < -PI {
                state.yaw += TAU;
            }
            if state.pitch > PI {
                state.pitch -= TAU;
            }
            if state.pitch < -PI {
                state.pitch += TAU;
            }
        }

        if total_pan != Vec2::ZERO {
            any = true;
            let radius = state.radius;
            state.center += transform.right() * total_pan.x * radius;
            state.center += transform.up() * total_pan.y * radius;
        }

        if any || state.is_added() {
            transform.rotation = Quat::from_euler(EulerRot::YXZ, state.yaw, state.pitch, 0.0);
            transform.translation = state.center + transform.back() * state.radius;
        }
    }
}
