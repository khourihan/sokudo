use serde::{Deserialize, Serialize};

use super::transform::WriteTransform;

#[derive(Serialize, Deserialize)]
pub struct WriteCollider {
    pub id: u32,
    pub transform: WriteTransform,
}
