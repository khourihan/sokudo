use glam::Vec3;
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::collider::Collider;

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,
    pub inspector: InspectElements,
}

impl World {
    pub fn initialize(&mut self) {
        for collider in self.colliders.iter_mut() {
            collider.set_starting_points();
        }
    }

    pub fn step(&mut self) {
        self.inspector.reset();

        for i in 0..self.colliders.len() {
            for j in 0..self.colliders.len() {
                if i >= j {
                    continue;
                }

                unsafe {
                    let collider1 = &mut *(self.colliders.get_unchecked_mut(i) as *mut Collider);
                    let collider2 = &mut *(self.colliders.get_unchecked_mut(j) as *mut Collider);

                    collider1.collide(collider2, &mut self.inspector);
                }
            }
        }

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
            inspector: self.inspector.clone(),
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
            inspector: InspectElements::default(),
        }
    }
}
