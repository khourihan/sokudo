use serde::Deserialize;

use crate::read::transform::ParsedTransform;

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub struct ParsedCollider {
    #[serde(skip)]
    pub id: u32,
    pub transform: ParsedTransform,
    pub shape: ParsedShape,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}
