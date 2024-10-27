use std::{ops::{Index, IndexMut}, slice::SliceIndex};

use glam::Vec3;
use sokudo_io::{read::collider::{ParsedCollider, ParsedColliderBody}, write::{collider::WriteCollider, inspect::InspectElements, transform::WriteTransform}};

use crate::{particle::Particle, rigid_body::RigidBody};

#[derive(Debug)]
pub struct Collider {
    /// This collider's unique ID.
    pub id: u32,
    /// The body of this collider.
    pub body: ColliderBody,
    /// Whether or not this collider is locked. 
    /// This turns off gravity and gives it infinite mass. 
    pub locked: bool,

    /// The position of the collider. For rigid bodies, this is located at its center of mass.
    pub position: Vec3,
    pub previous_position: Vec3,
    pub velocity: Vec3,
    pub previous_velocity: Vec3,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ColliderId(pub u32);

impl ColliderId {
    pub fn new(i: usize) -> ColliderId {
        ColliderId(i as u32)
    }
}

#[derive(Debug)]
pub enum ColliderBody {
    Particle(Particle),
    Rigid(RigidBody),
}

impl ColliderBody {
    #[inline]
    pub fn mass(&self) -> f32 {
        match self {
            ColliderBody::Particle(particle) => particle.mass,
            ColliderBody::Rigid(rb) => rb.mass,
        }
    }
}

impl From<ParsedCollider> for Collider {
    fn from(value: ParsedCollider) -> Self {
        Collider {
            id: value.id,
            locked: value.locked,
            body: value.body.into(),

            position: value.position,
            previous_position: value.position,
            velocity: value.velocity,
            previous_velocity: value.velocity,
        }
    }
}

impl From<ParsedColliderBody> for ColliderBody {
    fn from(value: ParsedColliderBody) -> Self {
        match value {
            ParsedColliderBody::Particle(particle) => ColliderBody::Particle(particle.into()),
            ParsedColliderBody::RigidBody(rb) => ColliderBody::Rigid(rb.into()),
        }
    }
}

impl From<&Collider> for WriteCollider {
    fn from(value: &Collider) -> Self {
        let transform = match &value.body {
            ColliderBody::Particle(_) => WriteTransform::from_translate(value.position),
            ColliderBody::Rigid(rb) => WriteTransform {
                translate: value.position,
                rotate: rb.rotation,
            }
        };
        
        WriteCollider {
            id: value.id,
            transform,
        }
    }
}

impl std::hash::Hash for Collider {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u32(self.id);
    }
}
