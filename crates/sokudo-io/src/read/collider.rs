use glam::{Quat, UVec3, Vec3};
use serde::Deserialize;

use crate::read::defaults::DefaultOptions;

use super::types::RawRotation;

#[derive(Debug)]
pub struct ParsedCollider {
    pub id: u32,
    pub body: ParsedColliderBody,
    pub locked: bool,

    pub material: ParsedMaterial,
    
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

        #[serde(default)]
        material: ParsedMaterial,
        #[serde(default = "DefaultOptions::mass")]
        mass: f32,
    },
    RigidBody {
        #[serde(default)]
        locked: bool,
        #[serde(default)]
        position: Vec3,
        #[serde(default)]
        velocity: Vec3,

        #[serde(default)]
        rotation: RawRotation,
        #[serde(default)]
        angular_velocity: Vec3,

        #[serde(default)]
        material: ParsedMaterial,
        #[serde(default = "DefaultOptions::mass")]
        mass: f32,

        shape: ParsedShape,
        #[serde(default = "DefaultOptions::scale")]
        scale: Vec3,
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
                material: _,
                mass,
            } => ParsedColliderBody::Particle(ParsedParticle {
                mass,
            }),
            RawCollider::RigidBody {
                locked: _,
                position: _,
                velocity: _,
                material: _,
                rotation,
                scale,
                angular_velocity,
                shape,
                mass,
                vertex_resolution,
                vertices,
            } => ParsedColliderBody::RigidBody(ParsedRigidBody {
                shape,
                rotation: rotation.into(),
                scale,
                angular_velocity,
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
    pub rotation: Quat,
    pub scale: Vec3,
    pub angular_velocity: Vec3,
    pub mass: f32,
    pub vertex_resolution: UVec3,
    pub vertices: Vec<Vec3>,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename = "Material")]
pub struct ParsedMaterial {
    #[serde(default = "DefaultOptions::material_restitution")]
    pub restitution: f32,
}

impl Default for ParsedMaterial {
    fn default() -> Self {
        Self {
            restitution: DefaultOptions::material_restitution(),
        }
    }
}
