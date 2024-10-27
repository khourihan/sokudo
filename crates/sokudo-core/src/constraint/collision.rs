use crate::collider::{Collider, ColliderBody, ColliderId};

use super::{compute_lagrange_update, Constraint};

pub struct ParticleCollisionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub compliance: f32,
}

impl Constraint<2> for ParticleCollisionConstraint {
    fn bodies(&self) -> [ColliderId; 2] {
        [self.particle, self.rb]
    }

    fn solve(&self, bodies: [&mut Collider; 2], dt: f32) {
        let [particle, rb] = bodies;
        
        let ColliderBody::Particle(particle_body) = &particle.body else {
            return;
        };

        let ColliderBody::Rigid(rb_body) = &rb.body else {
            return;
        };

        let depth = rb_body.sd(particle.position).abs();
        let n = -rb_body.sd_gradient(particle.position) * rb_body.sd(particle.position);
        
        // TODO: Compute generalized inverse mass for rigid body
        let w1 = 1.0 / particle_body.mass;
        let w2 = 1.0 / rb_body.mass;

        let gradients = [n, -n];
        let w = [w1, w2];

        let delta_lagrange = compute_lagrange_update(0.0, depth, gradients, w, self.compliance, dt);
        let d = delta_lagrange * n;

        match (particle.locked, rb.locked) {
            (true, true) => (),
            (false, false) => {
                particle.position -= d * 0.5;
                rb.position += d * 0.5;
            },
            (false, true) => {
                particle.position -= d;
            },
            (true, false) => {
                rb.position += d;
            },
        }
    }
}

pub struct RigidBodyCollisionConstraint {
    pub a: ColliderId,
    pub b: ColliderId,

    pub compliance: f32,
}
