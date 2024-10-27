use crate::{collider::{Collider, ColliderBody, ColliderId}, contact::ParticleContact};

use super::VelocityConstraint;

pub struct ParticleRestitutionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub contact: ParticleContact,
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
        
        let ColliderBody::Rigid(ref mut rb_body) = rb.body else {
            return;
        };

        let n = self.contact.normal;

        let rb_previous_velocity = rb_body.previous_angular_velocity.cross(self.contact.anchor2) + rb.previous_velocity;
        let vdiff_prev = particle.previous_velocity - rb_previous_velocity;
        let vn_prev = n.dot(vdiff_prev);

        let rb_velocity = rb_body.angular_velocity.cross(self.contact.anchor2) + rb.velocity;
        let vdiff = particle.velocity - rb_velocity;
        let vn = n.dot(vdiff);
        
        let w1 = if particle.locked { 0.0 } else { particle_body.inverse_mass() };
        let w2 = if rb.locked { 0.0 } else { rb_body.positional_inverse_mass(self.contact.anchor2, n) };
        let w_sum = w1 + w2;

        let impulse = n * ((-vn - self.coefficient * vn_prev) / w_sum);

        particle.velocity += impulse * w1;
        rb.velocity -= impulse * w2;

        rb_body.angular_velocity -= rb_body.global_inverse_inertia() * self.contact.anchor2.cross(impulse);
    }
}
