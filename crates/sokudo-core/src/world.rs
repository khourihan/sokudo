use glam::Vec3;
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::{collider::{Collider, ColliderBody, ColliderId}, constraint::{collision::ParticleCollisionConstraint, Constraint}, rigid_body::RigidBody};

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,

    pub constraints: Vec<Box<dyn Constraint>>,
    pub collision_constraints: Vec<Box<dyn Constraint>>,
    pub lagrange: Vec<f32>,

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
            collider.previous_velocity = collider.velocity;
        }

        // TODO: Narrow phase

        self.create_collisions();
        self.lagrange = vec![0.0; self.constraints.len() + self.collision_constraints.len()];
        self.solve_constraints();

        // Update velocities
        for collider in self.colliders.iter_mut() {
            collider.velocity = (collider.position - collider.previous_position) / self.dt;
        }

        // TODO: Solve velocity constraints

        self.sync_transforms();
    }

    fn solve_constraints(&mut self) {
        for (constraint, lagrange) in self.constraints.iter().chain(self.collision_constraints.iter()).zip(self.lagrange.iter_mut()) {
            let bodies: Vec<_> = unsafe {
                constraint.bodies().into_iter()
                    .map(|id| self.colliders.get_unchecked(id.0 as usize))
                    .collect()
            };

            let c = constraint.c(&bodies);
            let gradients = constraint.c_gradients(&bodies);
            let inverse_masses = constraint.inverse_masses(&bodies);

            let w_sum = inverse_masses
                .iter()
                .zip(gradients.iter())
                .fold(0.0, |acc, (&w, &g)| acc + w * g.length_squared());

            let delta_lagrange = if w_sum > f32::EPSILON {
                let tilde_compliance = constraint.compliance() / (self.dt * self.dt);
                (-c - tilde_compliance * *lagrange) / (w_sum + tilde_compliance)
            } else {
                0.0
            };

            *lagrange += delta_lagrange;

            let bodies = unsafe {
                constraint.bodies().into_iter()
                    .map(|id| &mut *(self.colliders.get_unchecked_mut(id.0 as usize) as *mut Collider))
                    .zip(gradients.into_iter())
                    .zip(inverse_masses.into_iter())
            };

            for ((body, gradient), inv_mass) in bodies {
                body.position += delta_lagrange * inv_mass * gradient;
            }
        }
    }

    fn create_collisions(&mut self) {
        self.collision_constraints.clear();

        for i in 0..self.colliders.len() {
            for j in 0..self.colliders.len() {
                if i >= j {
                    continue;
                }

                let id_a = ColliderId::new(i);
                let id_b = ColliderId::new(j);

                let a = unsafe { self.colliders.get_unchecked(i) };
                let b = unsafe { self.colliders.get_unchecked(j) };

                match (&a.body, &b.body) {
                    (ColliderBody::Particle(_), ColliderBody::Particle(_)) => (),
                    (ColliderBody::Particle(_particle), ColliderBody::Rigid(rb)) => {
                        if rb.sd(a.position) > 0.0 {
                            continue;
                        }

                        let constraint = ParticleCollisionConstraint {
                            particle: id_a,
                            rb: id_b,
                            compliance: 0.0,
                        };

                        self.collision_constraints.push(Box::new(constraint));
                    },
                    (ColliderBody::Rigid(rb), ColliderBody::Particle(_particle)) => {
                        if rb.sd(b.position) > 0.0 {
                            continue;
                        }

                        let constraint = ParticleCollisionConstraint {
                            particle: id_b,
                            rb: id_a,
                            compliance: 0.0,
                        };

                        self.collision_constraints.push(Box::new(constraint));
                    },
                    (ColliderBody::Rigid(_rb1), ColliderBody::Rigid(_rb2)) => {
                        todo!();
                    },
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

            constraints: Vec::new(),
            collision_constraints: Vec::new(),
            lagrange: Vec::new(),

            inspector: InspectElements::default(),
        }
    }
}
