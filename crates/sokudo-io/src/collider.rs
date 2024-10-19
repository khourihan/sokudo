use serde::Deserialize;

use crate::transform::ParsedTransform;

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub struct ParsedCollider {
    pub transform: ParsedTransform,
    pub shape: ParsedColliderShape,
}

#[derive(Deserialize, Debug)]
pub enum ParsedColliderShape {
    Cuboid,
}
