use std::f32::consts::{FRAC_PI_3, FRAC_PI_4, FRAC_PI_6};

use bevy::{prelude::*, utils::HashMap};
use bevy_mod_picking::PickableBundle;
use sokudo_io::{
    read::{
        collider::{ParsedColliderBody, ParsedShape},
        ParsedWorld,
    },
    write::{inspect::InspectFeature, ReadWorldStateHistory},
};

use crate::{camera::PanOrbitState, util::ToBevyType};

pub struct PlayerPlugin;

impl Plugin for PlayerPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderEntities>()
            .init_resource::<WorldStateIndex>()
            .init_resource::<DeltaTime>()
            .init_resource::<PlaybackTime>()
            .init_state::<PlayerState>()
            .add_systems(Startup, (setup_lights, setup_initial_state))
            .add_systems(
                PreUpdate,
                (
                    set_player_state_playing.run_if(in_state(PlayerState::Paused)),
                    set_player_state_paused.run_if(in_state(PlayerState::Playing)),
                    update_world_state
                        .after(set_player_state_playing)
                        .run_if(in_state(PlayerState::Playing)),
                    step_state_on_pause
                        .after(set_player_state_paused)
                        .run_if(in_state(PlayerState::Paused)),
                    restart_player,
                ),
            )
            .add_systems(Update, (update_inspect_elements, update_colliders));
    }
}

#[derive(Resource)]
pub struct InitialWorld {
    pub world: ParsedWorld,
}

#[derive(Resource, Default)]
pub struct DeltaTime {
    pub dt: f32,
}

#[derive(Resource, Default)]
pub struct PlaybackTime {
    pub time: f32,
}

#[derive(Resource)]
pub struct WorldStateHistory {
    pub history: ReadWorldStateHistory,
}

#[derive(Resource, Default)]
pub struct WorldStateIndex {
    pub step: usize,
}

#[derive(States, Clone, PartialEq, Eq, Hash, Debug, Default)]
pub enum PlayerState {
    #[default]
    Paused,
    Playing,
}

#[derive(Resource, Default)]
struct ColliderEntities {
    map: HashMap<u32, Entity>,
}

#[derive(Component)]
struct Collider;

fn setup_lights(mut commands: Commands, mut ambient_light: ResMut<AmbientLight>) {
    ambient_light.brightness = 200.0;

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::WHITE,
            illuminance: 5000.0,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 2.0 * FRAC_PI_3, FRAC_PI_6, 0.0)),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::WHITE,
            illuminance: 2000.0,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 4.0 * FRAC_PI_3, FRAC_PI_3, 0.0)),
        ..default()
    });

    commands.spawn(DirectionalLightBundle {
        directional_light: DirectionalLight {
            color: Color::WHITE,
            illuminance: 500.0,
            ..default()
        },
        transform: Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.0, FRAC_PI_4, 0.0)),
        ..default()
    });
}

fn setup_initial_state(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut collider_entities: ResMut<ColliderEntities>,
    mut delta_time: ResMut<DeltaTime>,
    world: Res<InitialWorld>,
) {
    delta_time.dt = world.world.dt;

    for collider in world.world.colliders.iter() {
        match &collider.body {
            ParsedColliderBody::Particle(_) => {
                let mesh = Mesh::from(Sphere::new(0.1));
                let material = StandardMaterial::from_color(Color::srgba(0.0, 0.0, 1.0, 0.8));

                let entity = commands
                    .spawn((
                        PbrBundle {
                            mesh: meshes.add(mesh),
                            material: materials.add(material),
                            transform: Transform::from_translation(collider.position.to_bevy()),
                            ..default()
                        },
                        Collider,
                        PickableBundle::default(),
                    ))
                    .id();

                collider_entities.map.insert(collider.id, entity);
            },
            ParsedColliderBody::RigidBody(rb) => {
                let mesh: Mesh = match rb.shape {
                    ParsedShape::Cuboid => Cuboid::new(1.0, 1.0, 1.0).into(),
                };

                let material = StandardMaterial::from_color(Color::srgba(1.0, 0.0, 0.0, 0.8));

                let entity = commands
                    .spawn((
                        PbrBundle {
                            mesh: meshes.add(mesh),
                            material: materials.add(material),
                            transform: Transform {
                                translation: collider.position.to_bevy(),
                                rotation: rb.rotation.to_bevy(),
                                scale: rb.scale.to_bevy(),
                            },
                            ..default()
                        },
                        Collider,
                        PickableBundle::default(),
                    ))
                    .id();

                collider_entities.map.insert(collider.id, entity);
            },
        }
    }
}

fn set_player_state_playing(
    keys: Res<ButtonInput<KeyCode>>,
    mut next_state: ResMut<NextState<PlayerState>>,
    mut playback_time: ResMut<PlaybackTime>,
    time: Res<Time>,
) {
    if keys.just_pressed(KeyCode::Space) {
        next_state.set(PlayerState::Playing);
        playback_time.time = time.elapsed_seconds();
    }
}

fn set_player_state_paused(keys: Res<ButtonInput<KeyCode>>, mut next_state: ResMut<NextState<PlayerState>>) {
    if keys.just_pressed(KeyCode::Space) {
        next_state.set(PlayerState::Paused)
    }
}

fn update_world_state(
    mut index: ResMut<WorldStateIndex>,
    mut next_state: ResMut<NextState<PlayerState>>,
    history: Res<WorldStateHistory>,
    delta_time: Res<DeltaTime>,
    mut playback_time: ResMut<PlaybackTime>,
    time: Res<Time>,
) {
    if time.elapsed_seconds() - playback_time.time <= delta_time.dt {
        return;
    }

    playback_time.time += delta_time.dt;

    index.step += 1;

    if history.history.len() <= index.step {
        next_state.set(PlayerState::Paused);
        index.step = 0;
    }
}

fn update_colliders(
    collider_entities: Res<ColliderEntities>,
    mut colliders: Query<&mut Transform, With<Collider>>,
    index: Res<WorldStateIndex>,
    history: Res<WorldStateHistory>,
) {
    if !index.is_changed() {
        return;
    }

    let world_state = history.history.get(index.step);

    for collider in world_state.colliders.iter() {
        let Some(&entity) = collider_entities.map.get(&collider.id) else {
            continue;
        };

        let Ok(mut transform) = colliders.get_mut(entity) else {
            continue;
        };

        transform.translation = collider.transform.translate.to_bevy();
        transform.rotation = collider.transform.rotate.to_bevy();
    }
}

fn update_inspect_elements(
    mut gizmos: Gizmos,
    index: Res<WorldStateIndex>,
    history: Res<WorldStateHistory>,
    cameras: Query<&Transform, With<PanOrbitState>>,
) {
    let world_state = history.history.get(index.step);
    let camera_transform = cameras.single();
    let camera = camera_transform.translation;

    for elem in world_state.inspector.elements.values() {
        match elem {
            InspectFeature::Point(point) => {
                let point = point.to_bevy();
                gizmos.circle(
                    point,
                    Dir3::new(camera - point).unwrap(),
                    0.01,
                    Color::srgb(1.0, 0.0, 0.0),
                );
            },
            InspectFeature::Ray { origin, direction } => {
                gizmos.ray(origin.to_bevy(), direction.to_bevy(), Color::srgb(0.0, 0.0, 1.0));
            },
            InspectFeature::Line { p1, p2 } => {
                gizmos.line(p1.to_bevy(), p2.to_bevy(), Color::srgb(0.0, 1.0, 0.0));
            },
        }
    }
}

fn step_state_on_pause(
    keys: Res<ButtonInput<KeyCode>>,
    mut index: ResMut<WorldStateIndex>,
    history: Res<WorldStateHistory>,
) {
    if keys.just_pressed(KeyCode::KeyJ) && index.step > 0 {
        index.step -= 1;
    }

    if keys.just_pressed(KeyCode::KeyL) && index.step < history.history.len() - 1 {
        index.step += 1;
    }
}

fn restart_player(keys: Res<ButtonInput<KeyCode>>, mut index: ResMut<WorldStateIndex>) {
    if keys.just_pressed(KeyCode::KeyR) {
        index.step = 0;
    }
}
