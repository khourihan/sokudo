use glam::Vec3;

use crate::collider::{Collider, ColliderBody, ColliderId};

use super::Constraint;

pub struct ParticleCollisionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub compliance: f32,
}

impl Constraint for ParticleCollisionConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.particle, self.rb]
    }

    fn c(&self, bodies: &[&Collider]) -> f32 {
        let [particle, rb] = *bodies else { return 0.0 };

        let ColliderBody::Rigid(ref rb_body) = rb.body else {
            return 0.0;
        };

        rb_body.sd(particle.position).abs()
    }

    fn c_gradients(&self, bodies: &[&Collider]) -> Vec<Vec3> {
        let [particle, rb] = *bodies else { return vec![] };

        let ColliderBody::Rigid(ref rb_body) = rb.body else {
            return vec![];
        };

        let n = rb_body.sd_gradient(particle.position);
        vec![-n, n]
    }

    fn inverse_masses(&self, bodies: &[&Collider]) -> Vec<f32> {
        let [particle, rb] = *bodies else { return vec![] };

        let ColliderBody::Particle(ref particle_body) = particle.body else {
            return vec![];
        };

        let ColliderBody::Rigid(ref rb_body) = rb.body else {
            return vec![];
        };

        // TODO: Correct generalized inverse mass for rigid body
        let w1 = if particle.locked { 0.0 } else { 1.0 / particle_body.mass };
        let w2 = if rb.locked { 0.0 } else { 1.0 / rb_body.mass };

        vec![w1, w2]
    }

    #[inline]
    fn compliance(&self) -> f32 {
        self.compliance
    }
}

pub struct RigidBodyCollisionConstraint {
    pub a: ColliderId,
    pub b: ColliderId,

    pub compliance: f32,
}
