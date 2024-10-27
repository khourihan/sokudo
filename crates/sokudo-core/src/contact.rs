use glam::Vec3;

use crate::{collider::{Collider, ColliderBody}, shape::AbstractShape};

// TODO: generalize for multiple contact points
#[derive(Clone, Debug, PartialEq)]
pub struct Contact {
    /// Contact point in global coordinates relative to the first body's center of mass.
    pub anchor1: Vec3,
    /// Contact point in global coordinates relative to the second body's center of mass.
    pub anchor2: Vec3,
    /// Contact normal in global coordinates.
    pub normal: Vec3,
    /// Penetration depth.
    pub depth: f32,
}

impl Contact {
    pub fn from_particle_rigid_body(
        particle: &Collider,
        rb: &Collider,
    ) -> Option<Contact> {
        let ColliderBody::Rigid(rb_body) = &rb.body else {
            return None;
        };

        let p_local_rb = (rb_body.rotation.inverse() * (particle.position - rb.position)) / rb_body.scale;
        let depth = rb_body.shape.sd(p_local_rb);

        if depth > 0.0 {
            return None;
        }

        let normal = rb_body.rotation * rb_body.shape.sd_gradient(p_local_rb);
        let point = particle.position;

        let anchor1 = Vec3::ZERO; // point - particle.position
        let anchor2 = point - rb.position;
        
        Some(Contact {
            anchor1,
            anchor2,
            normal,
            depth: depth.abs(),
        })
    }

    pub fn from_rigid_bodies(
        rb1: &Collider,
        rb2: &Collider,
    ) -> Option<Contact> {
        None
    }
}
