use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, WriteWorldState}};

use crate::collider::Collider;

pub struct World {
    pub colliders: Vec<Collider>,
}

impl World {
    pub fn step(&mut self) {

    }

    pub fn state(&self) -> WriteWorldState {
        WriteWorldState {
            colliders: self.colliders.iter().map(WriteCollider::from).collect(),
        }
    }
}

impl From<ParsedWorld> for World {
    fn from(value: ParsedWorld) -> Self {
        World {
            colliders: value.colliders.into_iter().map(Collider::from).collect(),
        }
    }
}
