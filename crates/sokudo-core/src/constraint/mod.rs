use distance::DistanceConstraint;
use glam::Vec3;
use sokudo_io::read::constraint::ParsedConstraint;
#[cfg(feature = "inspect")]
use sokudo_io::write::inspect::InspectElements;

use crate::collisions::collider::{Collider, ColliderId};

pub mod collision;
pub mod distance;

/// A constraint with a fixed number of participating bodies.
pub trait Constraint {
    /// The participating bodies of this constraint.
    fn bodies(&self) -> (ColliderId, ColliderId);

    /// Computes the constraint error (C).
    ///
    /// This value should be exactly zero when the constraint is satisfied.
    fn c(&self, a: &Collider, b: &Collider) -> f32;

    /// The gradient of the constraint (∇C) for each of the bodies.
    ///
    /// The direction of the gradient represents the direction in which C increases the most.
    /// The length of the gradient represents the amount by which C changes when moving its
    /// cooresponding body by one unit.
    fn c_gradients(&self, a: &Collider, b: &Collider) -> (Vec3, Vec3);

    /// The inverse masses of the participating bodies.
    fn inverse_masses(&self, a: &Collider, b: &Collider) -> (f32, f32);

    /// The anchors where positional impulses should be applied.
    fn anchors(&self, a: &Collider, b: &Collider) -> (Vec3, Vec3);

    /// The inverse stiffness of this constraint.
    fn compliance(&self) -> f32;
}

/// A constraint with an arbitrary number of participating bodies.
pub trait MultibodyConstraint {
    /// The participating bodies of this constraint.
    // TODO: Possibly remove need to allocate onto Vec<T>?
    fn bodies(&self) -> Vec<ColliderId>;

    /// Computes the constraint error (C).
    ///
    /// This value should be exactly zero when the constraint is satisfied.
    fn c(&self, bodies: &[&Collider]) -> f32;

    /// The gradient of the constraint (∇C) for each of the bodies.
    ///
    /// The direction of the gradient represents the direction in which C increases the most.
    /// The length of the gradient represents the amount by which C changes when moving its
    /// cooresponding body by one unit.
    fn c_gradients(&self, bodies: &[&Collider]) -> Vec<Vec3>;

    /// The inverse masses of the participating bodies.
    fn inverse_masses(&self, bodies: &[&Collider]) -> Vec<f32>;

    /// The anchors where positional impulses should be applied.
    fn anchors(&self, bodies: &[&Collider]) -> Vec<Vec3>;

    /// The inverse stiffness of this constraint.
    fn compliance(&self) -> f32;
}

pub trait VelocityConstraint {
    /// The participating bodies of this constraint.
    fn bodies(&self) -> (ColliderId, ColliderId);

    /// Solve the velocity constraint, applying the required impulses.
    fn solve(&self, a: &mut Collider, b: &mut Collider, #[cfg(feature = "inspect")] inspector: &mut InspectElements);
}

pub trait MultibodyVelocityConstraint {
    /// The participating bodies of this constraint.
    fn bodies(&self) -> Vec<ColliderId>;

    /// Solve the velocity constraint, applying the required impulses.
    fn solve(&self, bodies: std::vec::IntoIter<&mut Collider>);
}

impl From<ParsedConstraint> for Box<dyn Constraint> {
    fn from(value: ParsedConstraint) -> Self {
        Box::new(match value {
            ParsedConstraint::Distance {
                a,
                b,
                anchor1,
                anchor2,
                length,
                compliance,
            } => DistanceConstraint {
                a: ColliderId(a),
                b: ColliderId(b),
                l: length,
                anchor1,
                anchor2,
                compliance,
            },
        })
    }
}
