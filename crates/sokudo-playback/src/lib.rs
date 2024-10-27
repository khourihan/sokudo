use std::path;
use bevy::prelude::*;
use bevy_mod_picking::DefaultPickingPlugins;
use camera::PanOrbitPlugin;
use player::{InitialWorld, PlayerPlugin, WorldStateHistory};
use sokudo_io::{read::{ParseError, ParsedWorld}, write::{ReadStateError, ReadWorldStateHistory}};
use thiserror::Error;

mod player;
mod camera;

#[derive(Error, Debug)]
pub enum PlaybackError {
    /// A parse error.
    #[error(transparent)]
    Parse(#[from] ParseError),
    /// A state reading error.
    #[error(transparent)]
    ReadState(#[from] ReadStateError),
}

pub fn play<P>(world_path: P, history_path: P) -> Result<(), PlaybackError>
where
    P: AsRef<path::Path>,
{
    let world = ParsedWorld::read(world_path)?;
    let history = ReadWorldStateHistory::read(history_path)?;

    App::new()
        .add_plugins((DefaultPlugins, DefaultPickingPlugins, PanOrbitPlugin, PlayerPlugin))
        .insert_resource(WorldStateHistory { history })
        .insert_resource(InitialWorld { world })
        .run();

    Ok(())
}
