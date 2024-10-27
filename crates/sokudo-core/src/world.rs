use glam::{Quat, Vec3};
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::{collider::{Collider, ColliderBody, ColliderId}, constraint::{collision::ParticleCollisionConstraint, restitution::ParticleRestitutionConstraint, Constraint, VelocityConstraint}, contact::Contact, math::skew_symmetric_mat3, rigid_body::RigidBody};

pub struct World {
    pub steps: u32,
    pub dt: f32,
    pub gravity: Vec3,
    pub colliders: Vec<Collider>,

    pub constraints: Vec<Box<dyn Constraint>>,
    pub collision_constraints: Vec<Box<dyn Constraint>>,
    pub velocity_constraints: Vec<Box<dyn VelocityConstraint>>,
    pub velocity_collision_constraints: Vec<Box<dyn VelocityConstraint>>,
    pub lagrange: Vec<f32>,

    pub inspector: InspectElements,
}

impl World {
    pub fn initialize(&mut self) {
        for collider in self.colliders.iter_mut() {
            if let ColliderBody::Rigid(rb) = &mut collider.body {
                rb.compute_vertices();
                rb.compute_inertia_tensor();
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

            if let ColliderBody::Rigid(rb) = &mut collider.body {
                let external_torque = Vec3::ZERO;

                rb.previous_rotation = rb.rotation;

                let effective_angular_inertia = rb.global_inverse_inertia();
                let mut delta_ang_vel = self.dt * if effective_angular_inertia.is_finite() {
                    effective_angular_inertia.inverse() * external_torque
                } else {
                    Vec3::ZERO
                };

                // Solve for gyroscopic torque using a more stable and accurate implicit Euler
                // method.
                delta_ang_vel += {
                    let local_inertia = rb.inertia_tensor.tensor();

                    let local_ang_vel = rb.rotation.inverse() * rb.angular_velocity;
                    let angular_momentum = local_inertia * local_ang_vel;

                    let jacobian = local_inertia + self.dt
                        * (skew_symmetric_mat3(local_ang_vel) * local_inertia
                            - skew_symmetric_mat3(angular_momentum));

                    let f = self.dt * local_ang_vel.cross(angular_momentum);

                    let delta_ang_vel = -jacobian.inverse() * f;

                    rb.rotation * delta_ang_vel
                };

                rb.angular_velocity += delta_ang_vel;

                let delta_rot = Quat::from_scaled_axis(self.dt * rb.angular_velocity);
                rb.rotation = (delta_rot * rb.rotation).normalize();

                rb.previous_angular_velocity = rb.angular_velocity;
            }
        }

        // TODO: Narrow phase

        self.create_collisions();
        self.lagrange = vec![0.0; self.constraints.len() + self.collision_constraints.len()];
        self.solve_constraints();

        // Update velocities
        for collider in self.colliders.iter_mut() {
            collider.velocity = (collider.position - collider.previous_position) / self.dt;

            if let ColliderBody::Rigid(rb) = &mut collider.body {
                let delta_rot = rb.rotation * rb.previous_rotation.inverse();
                rb.angular_velocity = 2.0 * delta_rot.xyz() / self.dt;
                rb.angular_velocity = if delta_rot.w >= 0.0 { rb.angular_velocity } else { -rb.angular_velocity };
            }
        }

        self.solve_velocities();

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

            let anchors = constraint.anchors();

            let bodies = unsafe {
                constraint.bodies().into_iter()
                    .map(|id| &mut *(self.colliders.get_unchecked_mut(id.0 as usize) as *mut Collider))
                    .zip(gradients.into_iter())
                    .zip(inverse_masses.into_iter())
                    .zip(anchors.into_iter())
            };

            for (((body, gradient), inv_mass), anchor) in bodies {
                let p = delta_lagrange * gradient;
                body.position += p * inv_mass;

                if let ColliderBody::Rigid(rb) = &mut body.body {
                    rb.rotation = rb.rotation +
                        Quat::from_vec4(0.5 * (rb.global_inverse_inertia() * anchor.cross(p)).extend(0.0)) * rb.rotation;
                }
            }
        }
    }
    
    fn solve_velocities(&mut self) {
        for constraint in self.velocity_constraints.iter().chain(self.velocity_collision_constraints.iter()) {
            let bodies: Vec<_> = unsafe {
                constraint.bodies().into_iter()
                    .map(|id| &mut *(self.colliders.get_unchecked_mut(id.0 as usize) as *mut Collider))
                    .collect()
            };

            constraint.solve(bodies.into_iter());
        }
    }

    fn create_collisions(&mut self) {
        self.collision_constraints.clear();
        self.velocity_collision_constraints.clear();

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
                    (ColliderBody::Particle(_), ColliderBody::Rigid(_)) => {
                        let Some(contact) = Contact::from_particle_rigid_body(a, b) else {
                            continue;
                        };

                        let collision = ParticleCollisionConstraint {
                            particle: id_a,
                            rb: id_b,
                            contact: contact.clone(),
                            compliance: 0.0,
                        };

                        let restitution = ParticleRestitutionConstraint {
                            particle: id_a,
                            rb: id_b,
                            contact,
                            coefficient: 1.0,
                        };

                        self.collision_constraints.push(Box::new(collision));
                        self.velocity_collision_constraints.push(Box::new(restitution));
                    },
                    (ColliderBody::Rigid(_), ColliderBody::Particle(_)) => {
                        let Some(contact) = Contact::from_particle_rigid_body(b, a) else {
                            continue;
                        };

                        let collision = ParticleCollisionConstraint {
                            particle: id_b,
                            rb: id_a,
                            contact: contact.clone(),
                            compliance: 0.0,
                        };

                        let restitution = ParticleRestitutionConstraint {
                            particle: id_b,
                            rb: id_a,
                            contact,
                            coefficient: 1.0,
                        };

                        self.collision_constraints.push(Box::new(collision));
                        self.velocity_collision_constraints.push(Box::new(restitution));
                    },
                    (ColliderBody::Rigid(_rb1), ColliderBody::Rigid(_rb2)) => {
                        // todo!();
                    },
                }
            }
        }
    }

    fn sync_transforms(&mut self) {
        for collider in self.colliders.iter_mut() {
            if let ColliderBody::Rigid(rb) = &mut collider.body {
                // rb.transform.translate = collider.position;
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
            velocity_constraints: Vec::new(),
            velocity_collision_constraints: Vec::new(),
            lagrange: Vec::new(),

            inspector: InspectElements::default(),
        }
    }
}
