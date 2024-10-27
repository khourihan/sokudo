use crate::{collider::{Collider, ColliderBody, ColliderId}, contact::Contact};

use super::VelocityConstraint;

pub struct ParticleRestitutionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub contact: Contact,
    pub coefficient: f32,
}

impl VelocityConstraint for ParticleRestitutionConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.particle, self.rb]
    }

    fn solve(&self, mut bodies: std::vec::IntoIter<&mut Collider>) {
        let particle = bodies.next().unwrap();
        let rb = bodies.next().unwrap();

        let ColliderBody::Particle(ref particle_body) = particle.body else {
            return;
        };
        
        let ColliderBody::Rigid(ref rb_body) = rb.body else {
            return;
        };

        let n = self.contact.normal;

        let vdiff_prev = particle.previous_velocity - rb.previous_velocity;
        let vn_prev = n.dot(vdiff_prev);

        let vdiff = particle.velocity - rb.velocity;
        let vn = n.dot(vdiff);
        
        // TODO: Correct generalized inverse mass for rigid body
        let w1 = if particle.locked { 0.0 } else { 1.0 / particle_body.mass };
        let w2 = if rb.locked { 0.0 } else { 1.0 / rb_body.mass };
        let w_sum = w1 + w2;

        // TODO: Correct impulse for rigid body that accounts for angular velocity
        particle.velocity += n * (-vn - self.coefficient * vn_prev) * w1 / w_sum;
        rb.velocity -= n * (-vn - self.coefficient * vn_prev) * w2 / w_sum;
    }
}
