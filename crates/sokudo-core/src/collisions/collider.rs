use glam::Vec3;
use sokudo_io::{
    read::collider::{ParsedCollider, ParsedColliderBody, ParsedMaterial},
    write::{collider::WriteCollider, transform::WriteTransform},
};

use crate::{
    coefficient::{Coefficient, CoefficientCombine},
    collisions::{particle::Particle, rigid_body::RigidBody},
};

#[derive(Debug)]
pub struct Collider {
    /// This collider's unique ID.
    pub id: u32,
    /// The body of this collider.
    pub body: ColliderBody,
    /// Whether or not this collider is locked.
    /// This turns off gravity and gives it infinite mass.
    pub locked: bool,

    /// The material of this collider.
    pub material: Material,

    /// The position of the collider.
    pub position: Vec3,
    /// The required change in position for each substep. At the end of each step, this is added to `position`.
    pub delta_position: Vec3,
    pub previous_position: Vec3,
    pub velocity: Vec3,
    pub previous_velocity: Vec3,

    /// Damping on the collider's linear velocity.
    pub linear_damping: f32,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ColliderId(pub u32);

impl ColliderId {
    #[inline]
    pub fn new(i: usize) -> ColliderId {
        ColliderId(i as u32)
    }
}

#[derive(Debug)]
pub enum ColliderBody {
    Particle(Particle),
    Rigid(RigidBody),
}

#[derive(Debug)]
pub struct Material {
    pub restitution: Coefficient,
    pub dynamic_friction: Coefficient,
    pub stiffness: Coefficient,
}

impl Collider {
    /// The collider's center of mass, in global space.
    pub fn center_of_mass(&self) -> Vec3 {
        match &self.body {
            ColliderBody::Particle(_) => self.position,
            ColliderBody::Rigid(rb) => self.position + rb.rotation * rb.center_of_mass,
        }
    }
}

impl ColliderBody {
    #[inline]
    pub fn inverse_mass(&self) -> f32 {
        match self {
            ColliderBody::Particle(particle) => particle.inverse_mass,
            ColliderBody::Rigid(rb) => rb.inverse_mass,
        }
    }

    /// Computes the point relative to the body's center of mass in global space given a point
    /// relative to its center of mass in local space.
    #[inline]
    pub fn global_arm(&self, anchor: Vec3) -> Vec3 {
        match self {
            ColliderBody::Particle(_) => anchor,
            ColliderBody::Rigid(rb) => rb.rotation * anchor,
        }
    }

    /// Compute the generalized inverse mass of this rigid body at point `r` when applying
    /// positional correction along the vector `n`, where `r` is relative to the body's center of
    /// mass in global coordinates.
    #[inline]
    pub fn positional_inverse_mass(&self, r: Vec3, n: Vec3) -> f32 {
        match self {
            ColliderBody::Particle(particle) => particle.inverse_mass,
            ColliderBody::Rigid(rb) => rb.positional_inverse_mass(r, n),
        }
    }
}

impl From<ParsedCollider> for Collider {
    fn from(value: ParsedCollider) -> Self {
        Collider {
            id: value.id,
            locked: value.locked,
            body: {
                let mut body = ColliderBody::from(value.body);
                if value.locked {
                    match &mut body {
                        ColliderBody::Particle(particle) => particle.inverse_mass = 0.0,
                        ColliderBody::Rigid(rigid) => rigid.inverse_mass = 0.0,
                    }
                }
                body
            },

            material: value.material.into(),

            position: value.position,
            delta_position: Vec3::ZERO,
            previous_position: value.position,
            velocity: value.velocity,
            previous_velocity: value.velocity,

            linear_damping: value.linear_damping,
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

impl From<ParsedMaterial> for Material {
    fn from(value: ParsedMaterial) -> Self {
        Material {
            restitution: Coefficient::new(value.restitution, CoefficientCombine::Average),
            dynamic_friction: Coefficient::new(value.dynamic_friction, CoefficientCombine::Average),
            stiffness: Coefficient::new(value.stiffness, CoefficientCombine::Average),
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
            },
        };

        WriteCollider {
            id: value.id,
            transform,
        }
    }
}

impl std::hash::Hash for Collider {
    #[inline]
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u32(self.id);
    }
}
