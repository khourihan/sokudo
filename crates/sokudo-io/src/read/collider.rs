use glam::{UVec3, Vec3};
use serde::Deserialize;

use crate::read::{defaults::DefaultOptions, transform::ParsedTransform};

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
    #[serde(default = "DefaultOptions::mass")]
    pub mass: f32,
    #[serde(default)]
    pub locked: bool,

    #[serde(default = "DefaultOptions::vertex_resolution")]
    pub vertex_resolution: UVec3,
    #[serde(default)]
    pub vertices: Vec<Vec3>,

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
    #[serde(default = "DefaultOptions::material_roughness")]
    pub roughness: f32,
    #[serde(default = "DefaultOptions::material_resilience")]
    pub resilience: f32,
    #[serde(default = "DefaultOptions::material_hardness")]
    pub hardness: f32,

}

impl Default for ParsedMaterial {
    fn default() -> Self {
        Self {
            roughness: DefaultOptions::material_roughness(),
            resilience: DefaultOptions::material_resilience(),
            hardness: DefaultOptions::material_hardness(),
        }
    }
}
