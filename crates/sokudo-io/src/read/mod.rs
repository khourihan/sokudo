use std::{fs, io, path};

use collider::ParsedCollider;
use glam::Vec3;
use serde::Deserialize;
use thiserror::Error;

pub mod transform;
pub mod collider;
mod defaults;

#[derive(Error, Debug)]
pub enum ParseError {
    /// An IO error.
    #[error(transparent)]
    Io(#[from] io::Error),
    /// A RON error.
    #[error(transparent)]
    Ron(#[from] ron::de::SpannedError),
}

#[derive(Deserialize, Debug)]
#[serde(rename = "World")]
pub struct ParsedWorld {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,

    #[serde(default)]
    pub colliders: Vec<ParsedCollider>,
}

impl ParsedWorld {
    pub fn read<P>(path: P) -> Result<ParsedWorld, ParseError>
    where
        P: AsRef<path::Path>,
    {
        let file = fs::File::open(path)?;
        let mut world: ParsedWorld = ron::de::from_reader(file)?;
        
        for (id, collider) in world.colliders.iter_mut().enumerate() {
            collider.id = id as u32;
        }

        Ok(world)
    }
}
