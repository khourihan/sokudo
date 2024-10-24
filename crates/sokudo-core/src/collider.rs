use glam::Vec3;
use sokudo_io::{read::collider::{ParsedCollider, ParsedForces, ParsedMaterial}, write::collider::WriteCollider};

use crate::{shape::{AbstractShape, Shape}, transform::Transform};

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

    /// The precomputed starting points to test for intersections on this collider.
    starting_points: Vec<Vec3>,
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
    const MAX_GRADIENT_DESCENT_STEPS: u32 = 1000;
    const GRADIENT_DESCENT_STEP_SIZE: f32 = 0.1;
    const GRADIENT_DESCENT_EPSILON: f32 = 0.01 * 0.01;

    /// Simulates the collision between two [`Collider`]s, applying the necessary forces to resolve
    /// the collision if necessary.
    pub fn collide(&mut self, other: &mut Self) {
        let mut intersection = None;

        for point in self.starting_points().chain(other.starting_points()) {
            let closest = self.find_collision_point(other, point);
            if self.sd(closest) <= 0.0 && other.sd(closest) <= 0.0 {
                intersection = Some(closest);
                break;
            }
        }

        let Some(intersection) = intersection else {
            return;
        };

        let p_self = self.find_deepest_contact(other, intersection);
        let p_other = other.find_deepest_contact(self, intersection);

        let n_self = other.sd_gradient(p_self);
        let n_other = self.sd_gradient(p_other);

        let d_self = other.sd(p_self).abs();
        let d_other = self.sd(p_other).abs();

        let f_self = n_self * d_self * 100.0;
        let f_other = n_other * d_other * 100.0;

        self.apply_force(p_self, f_self - f_other);
        other.apply_force(p_other, f_other - f_self);
    }

    /// Incrementally finds an intersection point or closest point between two [`Collider`]s 
    /// using gradient descent starting at the given `point`.
    fn find_collision_point(&self, other: &Self, mut point: Vec3) -> Vec3 {
        let mut step = 0;
        let mut prev_point = point + 100.0;

        while (self.sd(point) > 0.0 || other.sd(point) > 0.0) 
            && (prev_point - point).length_squared() > Self::GRADIENT_DESCENT_EPSILON 
            && step < Self::MAX_GRADIENT_DESCENT_STEPS
        {
            prev_point = point;
            step += 1;

            if self.sd(point) > 0.0 {
                point -= self.sd_gradient(point) * self.sd(point);
            } else {
                point -= Self::GRADIENT_DESCENT_STEP_SIZE * other.sd_gradient(point);
                if self.sd(point) > 0.0 {
                    point -= self.sd_gradient(point) * self.sd(point);
                }
            }
        }

        point
    }

    /// Incrementally finds the deepest intersection of this collider in relation to `other` using
    /// gradient descent starting at the given `point`.
    fn find_deepest_contact(&self, other: &Self, mut point: Vec3) -> Vec3 {
        let mut step = 0;
        let mut prev_point = point + 100.0;

        while self.sd(point) <= 0.0 && other.sd(point) <= 0.0 
            && (prev_point - point).length_squared() > Self::GRADIENT_DESCENT_EPSILON
            && step < Self::MAX_GRADIENT_DESCENT_STEPS
        {
            prev_point = point;
            step += 1;

            point -= Self::GRADIENT_DESCENT_STEP_SIZE * other.sd_gradient(point);
            if self.sd(point) > 0.0 {
                point -= self.sd_gradient(point) * self.sd(point);
            }
        }

        point
    }

    pub fn set_starting_points(&mut self) {
        self.starting_points = self.shape.starting_points();
    }

    fn starting_points(&self) -> impl Iterator<Item = Vec3> + '_ {
        self.starting_points.iter().map(|&p| self.transform.globalize(p)) 
    }

    /// The signed distance from the given `point` to this [`Collider`], where the given `point` is
    /// in global space. 
    pub fn sd(&self, point: Vec3) -> f32 {
        self.shape.sd(self.transform.localize(point))
    }

    /// The gradient of the signed distance field of this [`Collider`] at the given `point`, where
    /// the given `point` is in global space.
    pub fn sd_gradient(&self, point: Vec3) -> Vec3 {
        let g = self.shape.sd_gradient(self.transform.localize(point));
        self.transform.globalize_direction(g)
    }

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

            starting_points: Vec::new(),
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
