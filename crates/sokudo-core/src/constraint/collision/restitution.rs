use glam::Vec3;

use crate::{collisions::{collider::{Collider, ColliderBody, ColliderId}, contact::{ContactData, PointContact}, rigid_body::RigidBody}, constraint::VelocityConstraint};

pub struct ParticleRestitutionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub rb_anchor: Vec3,
    pub normal: Vec3,
    pub coefficient: f32,
}

impl ParticleRestitutionConstraint {
    pub fn new(
        particle_id: ColliderId,
        rb_id: ColliderId,
        rb: &Collider,
        contact: &PointContact,
        coefficient: f32,
    ) -> Self {
        let rb_anchor = contact.point - rb.center_of_mass();

        Self {
            particle: particle_id,
            rb: rb_id,
            rb_anchor,
            normal: contact.normal,
            coefficient,
        }
    }
}

impl VelocityConstraint for ParticleRestitutionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.particle, self.rb)
    }

    fn solve(&self, a: &mut Collider, b: &mut Collider) {
        let ColliderBody::Particle(ref particle) = a.body else {
            return;
        };
        
        let ColliderBody::Rigid(ref mut rb) = b.body else {
            return;
        };

        let rb_previous_velocity = rb.previous_angular_velocity.cross(self.rb_anchor) + b.previous_velocity;
        let vdiff_prev = a.previous_velocity - rb_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let rb_velocity = rb.angular_velocity.cross(self.rb_anchor) + b.velocity;
        let vdiff = a.velocity - rb_velocity;
        let vn = self.normal.dot(vdiff);
        
        let w1 = if a.locked { 0.0 } else { particle.inverse_mass() };
        let w2 = if b.locked { 0.0 } else { rb.positional_inverse_mass(self.rb_anchor, self.normal) };
        let w_sum = w1 + w2;

        let impulse = self.normal * ((-vn - self.coefficient * vn_prev) / w_sum);

        a.velocity += impulse * w1;
        b.velocity -= impulse * w2;

        rb.angular_velocity -= rb.global_inverse_inertia() * self.rb_anchor.cross(impulse);
    }
}

pub struct RigidBodyRestitutionConstraint {
    pub a: ColliderId,
    pub b: ColliderId,

    pub normal: Vec3,
    pub anchor1: Vec3,
    pub anchor2: Vec3,

    pub coefficient: f32,
}

impl RigidBodyRestitutionConstraint {
    pub fn new(
        a_id: ColliderId,
        b_id: ColliderId,
        a_body: &RigidBody,
        b_body: &RigidBody,
        contact: &ContactData,
        coefficient: f32,
    ) -> Self {
        let anchor1 = a_body.rotation * (contact.point1 - a_body.center_of_mass);
        let anchor2 = b_body.rotation * (contact.point2 - b_body.center_of_mass);
        let normal = a_body.rotation * contact.normal1;

        Self {
            a: a_id,
            b: b_id,
            normal,
            anchor1,
            anchor2,
            coefficient,
        }
    }
}

impl VelocityConstraint for RigidBodyRestitutionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn solve(&self, a: &mut Collider, b: &mut Collider) {
        let ColliderBody::Rigid(ref mut rb1) = a.body else {
            return;
        };
        
        let ColliderBody::Rigid(ref mut rb2) = b.body else {
            return;
        };

        let rb1_previous_velocity = rb1.previous_angular_velocity.cross(self.anchor1) + a.previous_velocity;
        let rb2_previous_velocity = rb2.previous_angular_velocity.cross(self.anchor2) + b.previous_velocity;
        let vdiff_prev = rb1_previous_velocity - rb2_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let rb1_velocity = rb1.angular_velocity.cross(self.anchor1) + a.velocity;
        let rb2_velocity = rb2.angular_velocity.cross(self.anchor2) + b.velocity;
        let vdiff = rb1_velocity - rb2_velocity;
        let vn = self.normal.dot(vdiff);
        
        let w1 = if a.locked { 0.0 } else { rb1.positional_inverse_mass(self.anchor1, self.normal) };
        let w2 = if b.locked { 0.0 } else { rb2.positional_inverse_mass(self.anchor2, self.normal) };
        let w_sum = w1 + w2;

        let impulse = self.normal * ((-vn - self.coefficient * vn_prev) / w_sum);

        a.velocity += impulse * w1;
        b.velocity -= impulse * w2;

        rb1.angular_velocity += rb1.global_inverse_inertia() * self.anchor1.cross(impulse);
        rb2.angular_velocity -= rb2.global_inverse_inertia() * self.anchor2.cross(impulse);
    }
}
