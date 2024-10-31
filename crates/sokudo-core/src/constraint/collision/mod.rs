use glam::Vec3;

use crate::collisions::{collider::{Collider, ColliderId}, contact::{ContactData, PointContact}, rigid_body::RigidBody};

use super::Constraint;

pub mod restitution;

pub struct ParticleCollisionConstraint {
    pub particle: ColliderId,
    pub rb: ColliderId,

    pub rb_anchor: Vec3,
    pub depth: f32,
    pub normal: Vec3,
}

impl ParticleCollisionConstraint {
    pub fn new(
        particle_id: ColliderId,
        rb_id: ColliderId,
        rb: &Collider,
        contact: &PointContact,
    ) -> Self {
        let rb_anchor = contact.point - rb.center_of_mass();

        Self {
            particle: particle_id,
            rb: rb_id,
            rb_anchor,
            depth: contact.depth,
            normal: contact.normal,
        }
    }
}

impl Constraint for ParticleCollisionConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.particle, self.rb]
    }

    fn c(&self, _bodies: &[&Collider]) -> f32 {
        self.depth
    }

    fn c_gradients(&self, _bodies: &[&Collider]) -> Vec<Vec3> {
        let n = self.normal;
        vec![-n, n]
    }

    fn inverse_masses(&self, bodies: &[&Collider]) -> Vec<f32> {
        let [particle, rb] = *bodies else { return vec![] };

        let w1 = if particle.locked { 0.0 } else { particle.body.positional_inverse_mass(Vec3::ZERO, self.normal) };
        let w2 = if rb.locked { 0.0 } else { rb.body.positional_inverse_mass(self.rb_anchor, self.normal) };

        vec![w1, w2]
    }

    fn anchors(&self, _bodies: &[&Collider]) -> Vec<Vec3> {
        vec![Vec3::ZERO, self.rb_anchor]
    }

    #[inline]
    fn compliance(&self) -> f32 {
        0.0
    }
}

pub struct RigidBodyCollisionConstraint {
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
}

impl RigidBodyCollisionConstraint {
    pub fn new(
        a_id: ColliderId,
        b_id: ColliderId,
        a_body: &RigidBody,
        b_body: &RigidBody,
        contact: &ContactData,
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
        }
    }
}

impl Constraint for RigidBodyCollisionConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.a, self.b]
    }

    fn c(&self, _bodies: &[&Collider]) -> f32 {
        self.depth
    }

    fn c_gradients(&self, _bodies: &[&Collider]) -> Vec<Vec3> {
        vec![self.normal, -self.normal]
    }

    fn inverse_masses(&self, bodies: &[&Collider]) -> Vec<f32> {
        let [rb1, rb2] = *bodies else { return vec![] };

        let w1 = if rb1.locked { 0.0 } else { rb1.body.positional_inverse_mass(self.anchor1, self.normal) };
        let w2 = if rb2.locked { 0.0 } else { rb2.body.positional_inverse_mass(self.anchor2, self.normal) };

        vec![w1, w2]
    }

    #[inline]
    fn anchors(&self, _bodies: &[&Collider]) -> Vec<Vec3> {
        vec![self.anchor1, self.anchor2]
    }

    fn compliance(&self) -> f32 {
        0.0
    }
}
