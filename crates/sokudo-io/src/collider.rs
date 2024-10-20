use serde::Deserialize;

use crate::transform::ParsedTransform;

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub struct ParsedCollider {
    pub transform: ParsedTransform,
    pub shape: ParsedShape,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}
