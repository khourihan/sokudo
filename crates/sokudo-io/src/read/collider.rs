use glam::Vec3;
use serde::Deserialize;

use crate::read::transform::ParsedTransform;

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub struct ParsedCollider {
    #[serde(skip)]
    pub id: u32,
    pub shape: ParsedShape,
    #[serde(default)]
    pub transform: ParsedTransform,
    #[serde(default)]
    pub material: ParsedMaterial,
    #[serde(default = "default_mass")]
    pub mass: f32,
    #[serde(default)]
    pub locked: bool,

    #[serde(default)]
    pub forces: ParsedForces,
    #[serde(default)]
    pub velocity: Vec3,
    #[serde(default)]
    pub angular_velocity: Vec3,
    #[serde(default)]
    pub angular_momentum: Vec3,
    #[serde(default)]
    pub acceleration: Vec3,
}

fn default_mass() -> f32 { 1.0 }

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}

#[derive(Deserialize, Debug, Default)]
#[serde(rename = "Forces")]
pub struct ParsedForces {
    #[serde(default)]
    pub linear: Vec3,
    #[serde(default)]
    pub torque: Vec3,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Material")]
pub struct ParsedMaterial {
    pub roughness: f32,
    pub resilience: f32,
    pub hardness: f32,
}

impl Default for ParsedMaterial {
    fn default() -> Self {
        Self {
            roughness: 1.0,
            resilience: 0.2,
            hardness: 1.0,
        }
    }
}
