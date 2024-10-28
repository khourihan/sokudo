use glam::Vec3;

use crate::{collider::{Collider, ColliderBody, ColliderId}, shape::AbstractShape};

#[derive(Clone, Debug, PartialEq)]
pub struct ParticleContact {
    /// Contact point in global coordinates relative to the first body's center of mass.
    pub anchor1: Vec3,
    /// Contact point in global coordinates relative to the second body's center of mass.
    pub anchor2: Vec3,
    /// Contact normal in global coordinates.
    pub normal: Vec3,
    /// Penetration depth.
    pub depth: f32,
}

impl ParticleContact {
    pub fn new(
        particle: &Collider,
        rb: &Collider,
    ) -> Option<ParticleContact> {
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
        
        Some(ParticleContact {
            anchor1,
            anchor2,
            normal,
            depth: depth.abs(),
        })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VertexContact {
    /// Contact point in global coordinates relative to the first body's center of mass.
    pub anchor1: Vec3,
    /// Contact point in global coordinates relative to the second body's center of mass.
    pub anchor2: Vec3,
    /// Contact normal in global coordinates.
    pub normal: Vec3,
    /// Penetration depth.
    pub depth: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RigidBodyContact {
    pub contacts: Vec<(ColliderId, VertexContact)>,
}

impl RigidBodyContact {
    pub fn new(
        rb1: &Collider,
        rb2: &Collider,
        id_a: ColliderId,
        id_b: ColliderId,
    ) -> Option<RigidBodyContact> {
        let mut contact = RigidBodyContact { contacts: Vec::new() };

        contact.add_contact_vertices(rb1, rb2, id_a)?;
        contact.add_contact_vertices(rb2, rb1, id_b)?;

        if contact.contacts.is_empty() {
            return None;
        }

        Some(contact)
    }

    fn add_contact_vertices(&mut self, a: &Collider, b: &Collider, id_a: ColliderId) -> Option<()> {
        let ColliderBody::Rigid(a_body) = &a.body else {
            return None;
        };

        let ColliderBody::Rigid(b_body) = &b.body else {
            return None;
        };

        for &vertex in a_body.vertices.iter() {
            let point = (a_body.rotation * (a_body.scale * vertex)) + a.position;
            let p_local = (b_body.rotation.inverse() * (point - b.position)) / b_body.scale;
            
            let depth = b_body.shape.sd(p_local);
            
            if depth > 0.0 {
                continue;
            }

            let normal = b_body.rotation * b_body.shape.sd_gradient(p_local);

            let anchor1 = point - a.position;
            let anchor2 = point - b.position;

            self.contacts.push((id_a, VertexContact {
                anchor1,
                anchor2,
                normal,
                depth: depth.abs()
            }));
        }

        Some(())
    }
}
