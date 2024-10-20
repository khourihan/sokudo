use sokudo_io::{read::collider::ParsedCollider, write::collider::WriteCollider};

use crate::{shape::Shape, transform::Transform};

#[derive(Debug)]
pub struct Collider {
    pub id: u32,
    pub transform: Transform,
    pub shape: Shape,
}

impl From<ParsedCollider> for Collider {
    fn from(value: ParsedCollider) -> Self {
        Collider {
            id: value.id,
            transform: value.transform.into(),
            shape: value.shape.into(),
        }
    }
}

impl From<&Collider> for WriteCollider {
    fn from(value: &Collider) -> Self {
        WriteCollider {
            id: value.id,
            transform: (&value.transform).into(),
        }
    }
}

impl std::hash::Hash for Collider {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u32(self.id);
    }
}
