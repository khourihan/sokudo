use glam::Vec3;

use crate::{collisions::{collider::{Collider, ColliderBody, ColliderId}, contact::{ContactData, PointContact}, rigid_body::RigidBody}, constraint::VelocityConstraint};

pub struct RestitutionConstraint {
    pub a: ColliderId,
    pub b: ColliderId,

    pub normal: Vec3,
    pub anchor1: Vec3,
    pub anchor2: Vec3,

    pub coefficient: f32,
}

impl RestitutionConstraint {
    /// Create a new restitution constraint between a particle and a rigid body.
    pub fn new_particle_rb(
        particle_id: ColliderId,
        rb_id: ColliderId,
        rb: &Collider,
        contact: &PointContact,
        coefficient: f32,
    ) -> Self {
        let rb_anchor = contact.point - rb.center_of_mass();

        Self {
            a: particle_id,
            b: rb_id,
            anchor1: Vec3::ZERO,
            anchor2: rb_anchor,
            normal: -contact.normal,
            coefficient,
        }
    }

    /// Create a restitution constraint between two rigid bodies.
    pub fn new_rb_rb(
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

impl VelocityConstraint for RestitutionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn solve(&self, a: &mut Collider, b: &mut Collider) {
        let (rb1_previous_velocity, rb1_velocity) = match &a.body {
            ColliderBody::Particle(_) => (
                a.previous_velocity,
                a.velocity,
            ),
            ColliderBody::Rigid(rb) => (
                rb.previous_angular_velocity.cross(self.anchor1) + a.previous_velocity,
                rb.angular_velocity.cross(self.anchor1) + a.velocity,
            ),
        };

        let (rb2_previous_velocity, rb2_velocity) = match &b.body {
            ColliderBody::Particle(_) => (
                b.previous_velocity,
                b.velocity,
            ),
            ColliderBody::Rigid(rb) => (
                rb.previous_angular_velocity.cross(self.anchor2) + b.previous_velocity,
                rb.angular_velocity.cross(self.anchor2) + b.velocity,
            ),
        };

        let vdiff_prev = rb1_previous_velocity - rb2_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let vdiff = rb1_velocity - rb2_velocity;
        let vn = self.normal.dot(vdiff);

        let w1 = if a.locked { 0.0 } else { a.body.positional_inverse_mass(self.anchor1, self.normal) };
        let w2 = if b.locked { 0.0 } else { b.body.positional_inverse_mass(self.anchor2, self.normal) };
        let w_sum = w1 + w2;

        let impulse = self.normal * ((-vn - self.coefficient * vn_prev) / w_sum);

        a.velocity += impulse * w1;
        b.velocity -= impulse * w2;

        if let ColliderBody::Rigid(rb) = &mut a.body {
            rb.angular_velocity += rb.global_inverse_inertia() * self.anchor1.cross(impulse);
        }

        if let ColliderBody::Rigid(rb) = &mut b.body {
            rb.angular_velocity -= rb.global_inverse_inertia() * self.anchor2.cross(impulse);
        }
    }
}
