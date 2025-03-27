use std::{fs, io, path};

use collider::{ParsedCollider, ParsedColliderBody, RawCollider};
use constraint::ParsedConstraint;
use defaults::DefaultOptions;
use glam::Vec3;
use serde::Deserialize;
use thiserror::Error;

pub mod collider;
pub mod constraint;
mod defaults;
mod types;

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
pub(crate) struct RawWorld {
    steps: u32,
    #[serde(default = "DefaultOptions::dt")]
    dt: f32,
    #[serde(default = "DefaultOptions::gravity")]
    gravity: Vec3,
    #[serde(default = "DefaultOptions::constraint_iterations")]
    constraint_iterations: u32,
    #[serde(default = "DefaultOptions::substeps")]
    substeps: u32,

    #[serde(default)]
    colliders: Vec<RawCollider>,
    #[serde(default)]
    constraints: Vec<ParsedConstraint>,
}

#[derive(Debug)]
pub struct ParsedWorld {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub constraint_iterations: u32,
    pub substeps: u32,

    pub colliders: Vec<ParsedCollider>,
    pub constraints: Vec<ParsedConstraint>,
}

impl ParsedWorld {
    pub fn read<P>(path: P) -> Result<ParsedWorld, ParseError>
    where
        P: AsRef<path::Path>,
    {
        let file = fs::File::open(path)?;
        let raw_world: RawWorld = ron::de::from_reader(file)?;
        let world = ParsedWorld::from(raw_world);

        Ok(world)
    }
}

impl From<RawWorld> for ParsedWorld {
    fn from(raw: RawWorld) -> Self {
        ParsedWorld {
            steps: raw.steps,
            dt: raw.dt,
            gravity: raw.gravity,
            constraint_iterations: raw.constraint_iterations,
            substeps: raw.substeps,

            colliders: raw
                .colliders
                .into_iter()
                .enumerate()
                .map(|(i, collider)| ParsedCollider {
                    id: i as u32,
                    position: match collider {
                        RawCollider::Particle { position, .. } => position,
                        RawCollider::RigidBody { position, .. } => position,
                    },
                    velocity: match collider {
                        RawCollider::Particle { velocity, .. } => velocity,
                        RawCollider::RigidBody { velocity, .. } => velocity,
                    },
                    locked: match collider {
                        RawCollider::Particle { locked, .. } => locked,
                        RawCollider::RigidBody { locked, .. } => locked,
                    },
                    material: match collider {
                        RawCollider::Particle { material, .. } => material,
                        RawCollider::RigidBody { material, .. } => material,
                    },
                    linear_damping: match collider {
                        RawCollider::Particle { damping, .. } => damping,
                        RawCollider::RigidBody { damping, .. } => damping.linear,
                    },
                    body: ParsedColliderBody::from(collider),
                })
                .collect(),
            constraints: raw.constraints,
        }
    }
}
