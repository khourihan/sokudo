use glam::Vec3;
use sokudo_io::{read::collider::{ParsedCollider, ParsedForces, ParsedMaterial}, write::collider::WriteCollider};

use crate::{shape::Shape, transform::Transform};

#[derive(Debug)]
pub struct Collider {
    /// This collider's unique ID.
    pub id: u32,
    /// The transform of this collider. 
    ///
    /// Note that the collider's translation is equal to its center of mass.
    /// This means that, in local space, the collider's center of mass should be at the origin.
    pub transform: Transform,
    /// The shape of this collider.
    pub shape: Shape,
    /// The mass of this collider, in kilograms.
    pub mass: f32,
    /// The material of this collider.
    pub material: Material,
    /// Whether or not this collider is locked. 
    /// This turns off gravity and gives it infinite mass. 
    pub locked: bool,

    /// The forces acting on this collider.
    pub forces: Forces,
    /// The velocity of this collider, in m/s.
    pub velocity: Vec3,
    /// The angular velocity (encoded as a scaled axis) of this collider, in rad/s.
    pub angular_velocity: Vec3,
    /// The angular momentum (encoded as a scaled axis) of this collider, in rad/s.
    pub angular_momentum: Vec3,
    /// The acceleration of this collider, in m/s².
    pub acceleration: Vec3,
}

#[derive(Debug)]
pub struct Forces {
    /// The translational force on the object, in newtons. 
    pub linear: Vec3,
    /// The rotational force (encoded as a scaled axis) on the object, in newtons. 
    pub torque: Vec3,
}

#[derive(Debug)]
pub struct Material {
    pub roughness: f32,
    pub resilience: f32,
    pub hardness: f32,
}

impl Collider {
    /// Compute the velocity of this [`Collider`] at the given `point`.
    pub fn velocity_at_point(&self, point: Vec3) -> Vec3 {
        let arm = point - self.transform.translate;
        self.angular_velocity.cross(arm) + self.velocity
    }

    /// Apply the given force `f` to this [`Collider`] at the given `point`.
    ///
    /// `f`: N (newtons)
    pub fn apply_force(&mut self, point: Vec3, f: Vec3) {
        let arm = point - self.transform.translate;
        self.forces.linear += f;
        self.forces.torque += arm.cross(f);
    }
}

impl From<ParsedCollider> for Collider {
    fn from(value: ParsedCollider) -> Self {
        Collider {
            id: value.id,
            transform: value.transform.into(),
            shape: value.shape.into(),
            mass: value.mass,
            material: value.material.into(),
            locked: value.locked,

            forces: value.forces.into(),
            velocity: value.velocity,
            angular_velocity: value.angular_velocity,
            angular_momentum: value.angular_momentum,
            acceleration: value.acceleration,
        }
    }
}

impl From<&Collider> for WriteCollider {
    fn from(value: &Collider) -> Self {
        WriteCollider {
            id: value.id,
            transform: (&value.transform).into(),
        }
    }
}

impl From<ParsedForces> for Forces {
    fn from(value: ParsedForces) -> Self {
        Forces {
            linear: value.linear,
            torque: value.torque,
        }
    }
}

impl From<ParsedMaterial> for Material {
    fn from(value: ParsedMaterial) -> Self {
        Material {
            roughness: value.roughness,
            resilience: value.resilience,
            hardness: value.hardness,
        }
    }
}

impl std::hash::Hash for Collider {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        state.write_u32(self.id);
    }
}
