use glam::{Quat, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct WriteTransform {
    pub translate: Vec3,
    pub rotate: Quat,
}
