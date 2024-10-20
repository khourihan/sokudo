use serde::Serialize;

use super::transform::WriteTransform;

#[derive(Serialize)]
pub struct WriteCollider {
    pub id: u32,
    pub transform: WriteTransform,
}
