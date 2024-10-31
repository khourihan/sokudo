use glam::Vec3;

use crate::collisions::collider::{Collider, ColliderId};

use super::Constraint;

pub struct DistanceConstraint {
    /// The id of the first collider.
    pub a: ColliderId,
    /// The id of the second collider.
    pub b: ColliderId,

    /// The rest length of this distance constraint.
    pub l: f32,

    /// Anchor on the first collider in local space relative to its center of mass.
    pub anchor1: Vec3,
    /// Anchor on the second collider in local space relative to its center of mass.
    pub anchor2: Vec3,

    /// The inverse stiffness of this constraint.
    pub compliance: f32,
}

impl Constraint for DistanceConstraint {
    #[inline]
    fn bodies(&self) -> Vec<ColliderId> {
        vec![self.a, self.b]
    }

    // TODO: pass anchors in here
    fn c(&self, bodies: &[&Collider]) -> f32 {
        let [r1, r2] = self.anchors(bodies)[0..2] else { return 0.0 };

        (r1 - r2).length() - self.l
    }

    // TODO: pass anchors in here
    fn c_gradients(&self, bodies: &[&Collider]) -> Vec<Vec3> {
        let [r1, r2] = self.anchors(bodies)[0..2] else { return vec![] };

        let n = (r1 - r2).normalize();

        vec![n, -n]
    }

    // TODO: pass anchors in here
    fn inverse_masses(&self, bodies: &[&Collider]) -> Vec<f32> {
        let [a, b] = *bodies else { return vec![] };
        let [r1, r2] = self.anchors(bodies)[0..2] else { return vec![] };

        let n = (r1 - r2).normalize();

        let w1 = if a.locked { 0.0 } else { a.body.positional_inverse_mass(r1, n) };
        let w2 = if b.locked { 0.0 } else { b.body.positional_inverse_mass(r2, n) };

        vec![w1, w2]
    }

    #[inline]
    fn anchors(&self, bodies: &[&Collider]) -> Vec<Vec3> {
        let [a, b] = *bodies else { return vec![] };

        let r1 = a.body.global_arm(self.anchor1);
        let r2 = b.body.global_arm(self.anchor2);

        vec![r1, r2]
    }

    #[inline]
    fn compliance(&self) -> f32 {
        self.compliance
    }
}
