use glam::{Quat, UVec3, Vec3};
use parry3d::shape::{SharedShape, TypedShape};
use serde::Deserialize;

use crate::read::defaults::DefaultOptions;

use super::types::RawRotation;

#[derive(Debug)]
pub struct ParsedCollider {
    pub id: u32,
    pub body: ParsedColliderBody,
    pub locked: bool,

    pub material: ParsedMaterial,
    
    pub position: Vec3,
    pub velocity: Vec3,

    pub linear_damping: f32,
}

#[derive(Debug)]
pub enum ParsedColliderBody {
    Particle(ParsedParticle),
    RigidBody(ParsedRigidBody),
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Collider")]
pub(crate) enum RawCollider {
    Particle {
        #[serde(default)]
        locked: bool,
        #[serde(default)]
        position: Vec3,
        #[serde(default)]
        velocity: Vec3,

        #[serde(default)]
        material: ParsedMaterial,
        #[serde(default = "DefaultOptions::mass")]
        mass: f32,

        #[serde(default)]
        damping: f32,
    },
    RigidBody {
        #[serde(default)]
        locked: bool,
        #[serde(default)]
        position: Vec3,
        #[serde(default)]
        velocity: Vec3,

        #[serde(default)]
        rotation: RawRotation,
        #[serde(default)]
        angular_velocity: Vec3,

        #[serde(default)]
        material: ParsedMaterial,
        #[serde(default = "DefaultOptions::density")]
        density: f32,

        shape: ParsedShape,
        #[serde(default = "DefaultOptions::scale")]
        scale: Vec3,

        #[serde(default)]
        damping: ParsedRigidBodyDamping,
    },
}

impl From<RawCollider> for ParsedColliderBody {
    fn from(raw: RawCollider) -> Self {
        match raw {
            RawCollider::Particle {
                locked: _,
                position: _,
                velocity: _,
                material: _,
                damping: _,
                mass,
            } => ParsedColliderBody::Particle(ParsedParticle {
                mass,
            }),
            RawCollider::RigidBody {
                locked: _,
                position: _,
                velocity: _,
                material: _,
                rotation,
                scale,
                angular_velocity,
                shape,
                density,
                damping,
            } => ParsedColliderBody::RigidBody(ParsedRigidBody {
                shape,
                rotation: rotation.into(),
                scale,
                angular_velocity,
                density,
                angular_damping: damping.angular,
            }),
        }
    }
}

#[derive(Deserialize, Debug, Clone, Copy)]
#[serde(rename = "Material")]
pub struct ParsedMaterial {
    #[serde(default = "DefaultOptions::material_restitution")]
    pub restitution: f32,
}

impl Default for ParsedMaterial {
    fn default() -> Self {
        Self {
            restitution: DefaultOptions::material_restitution(),
        }
    }
}

#[derive(Debug)]
pub struct ParsedParticle {
    pub mass: f32,
}

#[derive(Debug)]
pub struct ParsedRigidBody {
    pub shape: ParsedShape,
    pub rotation: Quat,
    pub scale: Vec3,
    pub angular_velocity: Vec3,
    pub density: f32,
    pub angular_damping: f32,
}

#[derive(Deserialize, Debug)]
#[serde(rename = "Shape")]
pub enum ParsedShape {
    Cuboid,
}

#[derive(Deserialize, Debug, Clone, Copy, Default)]
#[serde(rename = "Damping")]
pub struct ParsedRigidBodyDamping {
    #[serde(default)]
    pub linear: f32,
    #[serde(default)]
    pub angular: f32,
}

impl ParsedShape {
    /// Returns `(unscaled, scaled)`
    pub fn to_scaled_shape(&self, scale: Vec3, n_subdivisions: u32) -> (SharedShape, SharedShape) {
        let unscaled = match self {
            ParsedShape::Cuboid => SharedShape::cuboid(0.5, 0.5, 0.5),
        };

        let scale = parry3d::math::Vector::<f32>::new(scale.x, scale.y, scale.z).abs();
        let scaled = match unscaled.as_typed_shape() {
            TypedShape::Ball(_) => todo!(),
            TypedShape::Cuboid(s) => SharedShape::new(s.scaled(&scale)),
            TypedShape::Capsule(_) => todo!(),
            TypedShape::Segment(_) => todo!(),
            TypedShape::Triangle(_) => todo!(),
            TypedShape::TriMesh(_) => todo!(),
            TypedShape::Polyline(_) => todo!(),
            TypedShape::HalfSpace(_) => todo!(),
            TypedShape::HeightField(_) => todo!(),
            TypedShape::Compound(_) => todo!(),
            TypedShape::ConvexPolyhedron(_) => todo!(),
            TypedShape::Cylinder(_) => todo!(),
            TypedShape::Cone(_) => todo!(),
            TypedShape::RoundCuboid(_) => todo!(),
            TypedShape::RoundTriangle(_) => todo!(),
            TypedShape::RoundCylinder(_) => todo!(),
            TypedShape::RoundCone(_) => todo!(),
            TypedShape::RoundConvexPolyhedron(_) => todo!(),
            TypedShape::Custom(_) => todo!(),
        };

        (unscaled, scaled)
    }
}
