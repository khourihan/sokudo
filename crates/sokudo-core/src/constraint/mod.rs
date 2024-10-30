use glam::Vec3;

use crate::collisions::collider::{Collider, ColliderId};

pub mod collision;

pub trait Constraint {
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
    fn anchors(&self) -> Vec<Vec3>;

    /// The inverse stiffness of this constraint.
    fn compliance(&self) -> f32;
}

pub trait VelocityConstraint {
    /// The participating bodies of this constraint.
    fn bodies(&self) -> Vec<ColliderId>;

    /// Solve the velocity constraint, applying the required impulses.
    fn solve(&self, bodies: std::vec::IntoIter<&mut Collider>);
}
