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

        let rb_previous_velocity = rb_body.previous_angular_velocity.cross(self.rb_anchor) + rb.previous_velocity;
        let vdiff_prev = particle.previous_velocity - rb_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let rb_velocity = rb_body.angular_velocity.cross(self.rb_anchor) + rb.velocity;
        let vdiff = particle.velocity - rb_velocity;
        let vn = self.normal.dot(vdiff);
        
        let w1 = if particle.locked { 0.0 } else { particle_body.inverse_mass() };
        let w2 = if rb.locked { 0.0 } else { rb_body.positional_inverse_mass(self.rb_anchor, self.normal) };
        let w_sum = w1 + w2;

        let impulse = self.normal * ((-vn - self.coefficient * vn_prev) / w_sum);

        particle.velocity += impulse * w1;
        rb.velocity -= impulse * w2;

        rb_body.angular_velocity -= rb_body.global_inverse_inertia() * self.rb_anchor.cross(impulse);
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

        let rb1_previous_velocity = rb1_body.previous_angular_velocity.cross(self.anchor1) + rb1.previous_velocity;
        let rb2_previous_velocity = rb2_body.previous_angular_velocity.cross(self.anchor2) + rb2.previous_velocity;
        let vdiff_prev = rb1_previous_velocity - rb2_previous_velocity;
        let vn_prev = self.normal.dot(vdiff_prev);

        let rb1_velocity = rb1_body.angular_velocity.cross(self.anchor1) + rb1.velocity;
        let rb2_velocity = rb2_body.angular_velocity.cross(self.anchor2) + rb2.velocity;
        let vdiff = rb1_velocity - rb2_velocity;
        let vn = self.normal.dot(vdiff);
        
        let w1 = if rb1.locked { 0.0 } else { rb1_body.positional_inverse_mass(self.anchor1, self.normal) };
        let w2 = if rb2.locked { 0.0 } else { rb2_body.positional_inverse_mass(self.anchor2, self.normal) };
        let w_sum = w1 + w2;

        let impulse = self.normal * ((-vn - self.coefficient * vn_prev) / w_sum);

        rb1.velocity += impulse * w1;
        rb2.velocity -= impulse * w2;

        rb1_body.angular_velocity += rb1_body.global_inverse_inertia() * self.anchor1.cross(impulse);
        rb2_body.angular_velocity -= rb2_body.global_inverse_inertia() * self.anchor2.cross(impulse);
    }
}
