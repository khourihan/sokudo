use crate::{shape::Shape, transform::Transform};

#[derive(Debug)]
pub struct Collider {
    pub transform: Transform,
    pub shape: Shape,
}
