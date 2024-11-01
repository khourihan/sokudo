use glam::Vec3;
use serde::Deserialize;

#[derive(Deserialize, Debug)]
#[serde(rename = "Constraint")]
pub enum ParsedConstraint {
    Distance {
        a: u32,
        b: u32,

        #[serde(default)]
        anchor1: Vec3,
        #[serde(default)]
        anchor2: Vec3,

        length: f32,
        compliance: f32,
    }
}
