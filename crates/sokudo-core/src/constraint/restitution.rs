use crate::{collider::{Collider, ColliderBody, ColliderId}, contact::{ParticleContact, VertexContact}};

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

pub struct RigidBodyRestitutionConstraint {
    pub a: ColliderId,
    pub b: ColliderId,

    pub contact: VertexContact,
    pub coefficient: f32,
}

impl VelocityConstraint for RigidBodyRestitutionConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.a, self.b]
    }

    fn solve(&self, mut bodies: std::vec::IntoIter<&mut Collider>) {
        let rb1 = bodies.next().unwrap();
        let rb2 = bodies.next().unwrap();

        let ColliderBody::Rigid(ref mut rb1_body) = rb1.body else {
            return;
        };
        
        let ColliderBody::Rigid(ref mut rb2_body) = rb2.body else {
            return;
        };

        let n = self.contact.normal;

        let rb1_previous_velocity = rb1_body.previous_angular_velocity.cross(self.contact.anchor1) + rb1.previous_velocity;
        let rb2_previous_velocity = rb2_body.previous_angular_velocity.cross(self.contact.anchor2) + rb2.previous_velocity;
        let vdiff_prev = rb1_previous_velocity - rb2_previous_velocity;
        let vn_prev = n.dot(vdiff_prev);

        let rb1_velocity = rb1_body.angular_velocity.cross(self.contact.anchor1) + rb1.velocity;
        let rb2_velocity = rb2_body.angular_velocity.cross(self.contact.anchor2) + rb2.velocity;
        let vdiff = rb1_velocity - rb2_velocity;
        let vn = n.dot(vdiff);
        
        let w1 = if rb1.locked { 0.0 } else { rb1_body.positional_inverse_mass(self.contact.anchor1, -n) };
        let w2 = if rb2.locked { 0.0 } else { rb2_body.positional_inverse_mass(self.contact.anchor2, n) };
        let w_sum = w1 + w2;

        let impulse = n * ((-vn - self.coefficient * vn_prev) / w_sum);

        rb1.velocity += impulse * w1;
        rb2.velocity -= impulse * w2;

        rb1_body.angular_velocity += rb1_body.global_inverse_inertia() * self.contact.anchor1.cross(impulse);
        rb2_body.angular_velocity -= rb2_body.global_inverse_inertia() * self.contact.anchor2.cross(impulse);
    }
}
