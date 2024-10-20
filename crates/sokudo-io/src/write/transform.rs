use glam::{Quat, Vec3};
use serde::Serialize;

#[derive(Serialize)]
pub struct WriteTransform {
    pub translate: Vec3,
    pub rotate: Quat,
}
