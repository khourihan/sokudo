use glam::{Mat3, Quat, Vec3};
use sokudo_io::{read::ParsedWorld, write::{collider::WriteCollider, inspect::InspectElements, WriteWorldState}};

use crate::{collisions::{collider::{Collider, ColliderBody, ColliderId}, contact_query, rigid_body::RigidBody}, constraint::{collision::{restitution::{ParticleRestitutionConstraint, RigidBodyRestitutionConstraint}, ParticleCollisionConstraint, RigidBodyCollisionConstraint}, Constraint, VelocityConstraint}, math::skew_symmetric_mat3};

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
                rb.compute_mass_properties();
            }
        }
    }

    pub fn step(&mut self) {
        self.inspector.reset();

        // TODO: Collect collision pairs

        self.integrate_velocities();
        // TODO: Warm start
        self.integrate_positions();
        // TODO: Relax

        // TODO: Narrow phase

        self.create_collisions();
        self.lagrange = vec![0.0; self.constraints.len() + self.collision_constraints.len()];
        self.solve_constraints();

        self.update_velocities();

        self.solve_velocities();
    }

    fn integrate_velocities(&mut self) {
        for collider in self.colliders.iter_mut().filter(|c| !c.locked) {
            let inv_mass = collider.body.inverse_mass();
            let gravity = self.gravity / inv_mass;
            let external_forces = gravity;

            if collider.linear_damping != 0.0 {
                collider.velocity *= 1.0 / (1.0 + self.dt * collider.linear_damping);
            }

            collider.velocity += self.dt * external_forces * inv_mass;
            collider.previous_velocity = collider.velocity;

            if let ColliderBody::Rigid(rb) = &mut collider.body {
                let external_torque = Vec3::ZERO;

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

                if rb.angular_damping != 0.0 {
                    rb.angular_velocity *= 1.0 / (1.0 + self.dt * rb.angular_damping);
                }

                rb.angular_velocity += delta_ang_vel;
                rb.previous_angular_velocity = rb.angular_velocity;
            }
        }
    }

    fn integrate_positions(&mut self) {
        for collider in self.colliders.iter_mut().filter(|c| !c.locked) {
            collider.previous_position = collider.position;
            collider.position += self.dt * collider.velocity;

            if let ColliderBody::Rigid(rb) = &mut collider.body {
                rb.previous_rotation = rb.rotation;

                let delta_rot = Quat::from_scaled_axis(self.dt * rb.angular_velocity);
                rb.rotation = (delta_rot * rb.rotation).normalize();
            }
        }
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

            let anchors = constraint.anchors(&bodies);

            let bodies = unsafe {
                constraint.bodies().into_iter()
                    .map(|id| &mut *(self.colliders.get_unchecked_mut(id.0 as usize) as *mut Collider))
                    .zip(gradients.into_iter())
                    .zip(inverse_masses.into_iter())
                    .zip(anchors.into_iter())
            };

            for (((body, gradient), inv_mass), anchor) in bodies {
                let p = delta_lagrange * gradient;
                body.delta_position += p * inv_mass;

                if let ColliderBody::Rigid(rb) = &mut body.body {
                    let inverse_inertia = if body.locked { Mat3::ZERO } else { rb.global_inverse_inertia() };
                    rb.rotation = Quat::normalize(rb.rotation +
                        Quat::from_vec4(0.5 * (inverse_inertia * anchor.cross(p)).extend(0.0)) * rb.rotation);
                }
            }
        }

        for collider in self.colliders.iter_mut() {
            collider.position += collider.delta_position;
            collider.delta_position = Vec3::ZERO;
        }
    }

    fn update_velocities(&mut self) {
        for collider in self.colliders.iter_mut() {
            collider.velocity = (collider.position - collider.previous_position) / self.dt;

            if let ColliderBody::Rigid(rb) = &mut collider.body {
                let delta_rot = rb.rotation * rb.previous_rotation.inverse();
                rb.angular_velocity = 2.0 * delta_rot.xyz() / self.dt;
                rb.angular_velocity = if delta_rot.w >= 0.0 { rb.angular_velocity } else { -rb.angular_velocity };
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

                let a = unsafe { &*(self.colliders.get_unchecked(i) as *const Collider) };
                let b = unsafe { &*(self.colliders.get_unchecked(j) as *const Collider) };

                match (&a.body, &b.body) {
                    (ColliderBody::Particle(_), ColliderBody::Particle(_)) => (),
                    (ColliderBody::Particle(_), ColliderBody::Rigid(rb)) => {
                        if self.add_particle_rb_collision(a, b, rb, id_a, id_b).is_none() {
                            continue;
                        };
                    },
                    (ColliderBody::Rigid(rb), ColliderBody::Particle(_)) => {
                        if self.add_particle_rb_collision(b, a, rb, id_b, id_a).is_none() {
                            continue;
                        };
                    },
                    (ColliderBody::Rigid(a_body), ColliderBody::Rigid(b_body)) => {
                        if self.add_rb_collision(a, b, a_body, b_body, id_a, id_b).is_none() {
                            continue;
                        };
                    },
                }
            }
        }
    }

    fn add_particle_rb_collision(
        &mut self,
        particle: &Collider,
        rb: &Collider,
        rb_body: &RigidBody,
        particle_id: ColliderId,
        rb_id: ColliderId,
    ) -> Option<()> {
        let contact = contact_query::contact_point(
            rb_body,
            rb.position,
            rb_body.rotation,
            particle.position,
        )?;

        let collision = ParticleCollisionConstraint::new(
            particle_id,
            rb_id,
            rb,
            &contact,
        );

        let restitution = ParticleRestitutionConstraint::new(
            particle_id,
            rb_id,
            rb,
            &contact,
            *rb.material.restitution.combine(particle.material.restitution),
        );

        self.collision_constraints.push(Box::new(collision));
        self.velocity_collision_constraints.push(Box::new(restitution));

        Some(())
    }

    fn add_rb_collision(
        &mut self,
        a: &Collider,
        b: &Collider,
        a_body: &RigidBody,
        b_body: &RigidBody,
        a_id: ColliderId,
        b_id: ColliderId,
    ) -> Option<()> {
        let effective_speculative_margin = {
            // TODO: clamp linear velocities to the maximum speculative margins.
            self.dt * (a.velocity - b.velocity).length()
        };

        // TODO: make this configurable
        let contact_tolerance = 0.01;
        let collision_margin_sum = 0.0;

        let max_contact_dist = effective_speculative_margin.max(contact_tolerance) + collision_margin_sum;

        let manifolds = contact_query::contact_manifolds(
            a_body,
            a.position,
            a_body.rotation,
            b_body,
            b.position,
            b_body.rotation,
            max_contact_dist,
        );

        if manifolds.is_empty() {
            return None;
        }

        for manifold in manifolds.iter() {
            for contact in manifold.contacts.iter() {
                let collision = RigidBodyCollisionConstraint::new(
                    a_id,
                    b_id,
                    a_body,
                    b_body,
                    contact,
                );

                let restitution = RigidBodyRestitutionConstraint::new(
                    a_id,
                    b_id,
                    a_body,
                    b_body,
                    contact,
                    *a.material.restitution.combine(b.material.restitution),
                );

                self.collision_constraints.push(Box::new(collision));
                self.velocity_collision_constraints.push(Box::new(restitution));
            }
        }

        Some(())
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
