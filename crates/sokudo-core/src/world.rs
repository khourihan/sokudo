use glam::Vec3;
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, WriteWorldState}};

use crate::collider::Collider;

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,
}

impl World {
    pub fn step(&mut self) {
        for collider in self.colliders.iter_mut() {
            // F = ma
            collider.acceleration = if collider.locked { Vec3::ZERO } else { collider.forces.linear / collider.mass };
            
            // Semi-implicit euler integration
            collider.velocity += collider.acceleration * self.dt;
            collider.transform.translate += collider.velocity * self.dt;

            // Reset forces
            collider.forces.linear = Vec3::ZERO;
        }
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
            steps: value.steps,
            dt: value.dt,
            gravity: value.gravity,
            colliders: value.colliders.into_iter().map(Collider::from).collect(),
        }
    }
}
