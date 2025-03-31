use glam::Vec3;

use crate::collisions::{
    collider::{Collider, ColliderBody, ColliderId},
    contact::{ContactData, PointContact},
    rigid_body::RigidBody,
};

use super::{Constraint, VelocityConstraint};

#[derive(Debug, Clone)]
pub struct CollisionConstraint {
    /// The id of the first body.
    pub a: ColliderId,
    /// The id of the second body.
    pub b: ColliderId,

    /// The contact point relative to the first body's center of mass in global space.
    pub anchor1: Vec3,
    /// The contact point relative to the second body's center of mass in global space.
    pub anchor2: Vec3,

    /// The depth of the penetration.
    pub depth: f32,
    /// The contact normal in global space, pointing from the first body to the second body.
    pub normal: Vec3,

    pub restitution: f32,
    pub friction: f32,
}

impl CollisionConstraint {
    /// Create a new collision constraint between a particle and a rigid body.
    pub fn new_particle_rb(
        particle_id: ColliderId,
        rb_id: ColliderId,
        rb: &Collider,
        contact: &PointContact,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let rb_anchor = contact.point - rb.center_of_mass();

        Self {
            a: particle_id,
            b: rb_id,
            anchor1: Vec3::ZERO,
            anchor2: rb_anchor,
            depth: contact.depth,
            normal: -contact.normal,
            restitution,
            friction,
        }
    }

    /// Create a new collision constraint between two rigid bodies.
    pub fn new_rb_rb(
        a_id: ColliderId,
        b_id: ColliderId,
        a_body: &RigidBody,
        b_body: &RigidBody,
        contact: &ContactData,
        restitution: f32,
        friction: f32,
    ) -> Self {
        let anchor1 = a_body.rotation * (contact.point1 - a_body.center_of_mass);
        let anchor2 = b_body.rotation * (contact.point2 - b_body.center_of_mass);
        let normal = a_body.rotation * contact.normal1;

        Self {
            a: a_id,
            b: b_id,
            anchor1,
            anchor2,
            depth: contact.depth,
            normal,
            restitution,
            friction,
        }
    }
}

impl Constraint for CollisionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn c(&self, _a: &Collider, _b: &Collider) -> f32 {
        self.depth
    }

    fn c_gradients(&self, _a: &Collider, _b: &Collider) -> (Vec3, Vec3) {
        (self.normal, -self.normal)
    }

    fn inverse_masses(&self, a: &Collider, b: &Collider) -> (f32, f32) {
        let w1 = if a.locked {
            0.0
        } else {
            a.body.positional_inverse_mass(self.anchor1, self.normal)
        };
        let w2 = if b.locked {
            0.0
        } else {
            b.body.positional_inverse_mass(self.anchor2, self.normal)
        };

        (w1, w2)
    }

    #[inline]
    fn anchors(&self, _a: &Collider, _b: &Collider) -> (Vec3, Vec3) {
        (self.anchor1, self.anchor2)
    }

    fn compliance(&self) -> f32 {
        0.0
    }
}

impl VelocityConstraint for CollisionConstraint {
    #[inline]
    fn bodies(&self) -> (ColliderId, ColliderId) {
        (self.a, self.b)
    }

    fn solve(&self, a: &mut Collider, b: &mut Collider) {
        let (rb1_previous_velocity, rb1_velocity) = match &a.body {
            ColliderBody::Particle(_) => (a.previous_velocity, a.velocity),
            ColliderBody::Rigid(rb) => (
                rb.previous_angular_velocity.cross(self.anchor1) + a.previous_velocity,
                rb.angular_velocity.cross(self.anchor1) + a.velocity,
            ),
        };

        let (rb2_previous_velocity, rb2_velocity) = match &b.body {
            ColliderBody::Particle(_) => (b.previous_velocity, b.velocity),
            ColliderBody::Rigid(rb) => (
                rb.previous_angular_velocity.cross(self.anchor2) + b.previous_velocity,
                rb.angular_velocity.cross(self.anchor2) + b.velocity,
            ),
        };

        let vdiff_prev = rb1_previous_velocity - rb2_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let vdiff = rb1_velocity - rb2_velocity;
        let vn = self.normal.dot(vdiff);

        let w1 = if a.locked {
            0.0
        } else {
            a.body.positional_inverse_mass(self.anchor1, self.normal)
        };
        let w2 = if b.locked {
            0.0
        } else {
            b.body.positional_inverse_mass(self.anchor2, self.normal)
        };
        let w_sum = w1 + w2;

        let restitution = (-self.restitution * vn_prev).min(0.0);
        let impulse = self.normal * ((-vn + restitution) / w_sum);

        a.velocity += impulse * w1;
        b.velocity -= impulse * w2;

        if let ColliderBody::Rigid(rb) = &mut a.body {
            rb.angular_velocity += rb.global_inverse_inertia() * self.anchor1.cross(impulse);
        }

        if let ColliderBody::Rigid(rb) = &mut b.body {
            rb.angular_velocity += rb.global_inverse_inertia() * self.anchor2.cross(-impulse);
        }
    }
}
