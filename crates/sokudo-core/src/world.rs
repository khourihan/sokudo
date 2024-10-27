use glam::Vec3;
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::{collider::{Collider, ColliderBody, ColliderId}, constraint::{collision::ParticleCollisionConstraint, Constraint}, rigid_body::RigidBody};

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,
    pub collision_constraints: Vec<ParticleCollisionConstraint>,
    pub inspector: InspectElements,
}

impl World {
    pub fn initialize(&mut self) {
        for collider in self.colliders.iter_mut() {
            if let ColliderBody::Rigid(rb) = &mut collider.body {
                rb.compute_vertices();
                rb.compute_moments();
            }
        }
    }

    pub fn step(&mut self) {
        self.inspector.reset();

        // TODO: Collect collision pairs

        // Integrate
        for collider in self.colliders.iter_mut().filter(|c| !c.locked) {
            let mass = collider.body.mass();
            let external_forces = self.gravity * mass;

            collider.previous_position = collider.position;
            collider.velocity += self.dt * external_forces / mass;
            collider.position += self.dt * collider.velocity;
        }

        // TODO: Narrow phase

        self.solve_collisions();
        // TODO: Solve position constraints

        // Update velocities
        for collider in self.colliders.iter_mut() {
            collider.velocity = (collider.position - collider.previous_position) / self.dt;
        }

        // TODO: Solve velocity constraints

        self.sync_transforms();
    }

    fn solve_collisions(&mut self) {
        self.collision_constraints.clear();

        for i in 0..self.colliders.len() {
            for j in 0..self.colliders.len() {
                if i == j {
                    continue;
                }

                let id_a = ColliderId::new(i);
                let id_b = ColliderId::new(j);

                unsafe {
                    let a = &mut *(self.colliders.get_unchecked_mut(i) as *mut Collider);
                    let b = &mut *(self.colliders.get_unchecked_mut(j) as *mut Collider);

                    match (&mut a.body, &mut b.body) {
                        (ColliderBody::Particle(_), ColliderBody::Particle(_)) => (),
                        (ColliderBody::Particle(particle), ColliderBody::Rigid(rb)) => {
                            if rb.sd(a.position) > 0.0 {
                                continue;
                            }

                            let constraint = ParticleCollisionConstraint {
                                particle: id_a,
                                rb: id_b,
                                compliance: 0.0,
                            };

                            constraint.solve([a, b], self.dt);

                            self.collision_constraints.push(constraint);
                        },
                        (ColliderBody::Rigid(rb), ColliderBody::Particle(particle)) => {
                            if rb.sd(b.position) > 0.0 {
                                continue;
                            }

                            let constraint = ParticleCollisionConstraint {
                                particle: id_b,
                                rb: id_a,
                                compliance: 0.0,
                            };

                            constraint.solve([b, a], self.dt);

                            self.collision_constraints.push(constraint);
                        },
                        (ColliderBody::Rigid(rb1), ColliderBody::Rigid(rb2)) => {

                        },
                    }
                }
            }
        }
    }

    fn sync_transforms(&mut self) {
        for collider in self.colliders.iter_mut() {
            if let ColliderBody::Rigid(rb) = &mut collider.body {
                rb.transform.translate = collider.position;
            }
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
            collision_constraints: Vec::new(),
            inspector: InspectElements::default(),
        }
    }
}
