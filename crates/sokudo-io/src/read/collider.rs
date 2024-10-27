use glam::{UVec3, Vec3};
use serde::{de::{EnumAccess, Error, MapAccess, VariantAccess, Visitor}, Deserialize, Deserializer};

use crate::read::{defaults::DefaultOptions, transform::ParsedTransform};

#[derive(Debug)]
pub struct ParsedCollider {
    pub id: u32,
    pub body: ParsedColliderBody,
    pub locked: bool,
    
    pub position: Vec3,
    pub velocity: Vec3,
}

#[derive(Debug)]
pub enum ParsedColliderBody {
    Particle(ParsedParticle),
    RigidBody(ParsedRigidBody),
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub(crate) enum RawCollider {
    Particle {
        #[serde(default)]
        locked: bool,
        #[serde(default)]
        position: Vec3,
        #[serde(default)]
        velocity: Vec3,

        #[serde(default = "DefaultOptions::mass")]
        mass: f32,
    },
    RigidBody {
        #[serde(default)]
        locked: bool,
        #[serde(default)]
        transform: ParsedTransform,
        #[serde(default)]
        velocity: Vec3,

        shape: ParsedShape,
        #[serde(default = "DefaultOptions::mass")]
        mass: f32,
        #[serde(default = "DefaultOptions::vertex_resolution")]
        vertex_resolution: UVec3,
        #[serde(default)]
        vertices: Vec<Vec3>,
    },
}

impl From<RawCollider> for ParsedColliderBody {
    fn from(raw: RawCollider) -> Self {
        match raw {
            RawCollider::Particle {
                locked: _,
                position: _,
                velocity: _,
                mass,
            } => ParsedColliderBody::Particle(ParsedParticle {
                mass,
            }),
            RawCollider::RigidBody {
                locked: _,
                transform,
                velocity: _,
                shape,
                mass,
                vertex_resolution,
                vertices,
            } => ParsedColliderBody::RigidBody(ParsedRigidBody {
                shape,
                transform,
                mass,
                vertex_resolution,
                vertices,
            }),
        }
    }
}

#[derive(Debug)]
pub struct ParsedParticle {
    pub mass: f32,
}

#[derive(Debug)]
pub struct ParsedRigidBody {
    pub shape: ParsedShape,
    pub transform: ParsedTransform,
    pub mass: f32,
    pub vertex_resolution: UVec3,
    pub vertices: Vec<Vec3>,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}
