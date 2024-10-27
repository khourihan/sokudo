use glam::Vec3;

use crate::collider::{Collider, ColliderId};

pub mod collision;

pub trait Constraint<const N_BODIES: usize> {
    fn bodies(&self) -> [ColliderId; N_BODIES];

    fn solve(&self, bodies: [&mut Collider; N_BODIES], dt: f32);
}

pub fn compute_lagrange_update<const N: usize>(
    lagrange: f32,
    c: f32,
    gradients: [Vec3; N],
    inv_masses: [f32; N],
    compliance: f32,
    dt: f32,
) -> f32 {
    let w_sum = inv_masses
        .iter()
        .enumerate()
        .fold(0.0, |acc, (i, &w)| acc + w * gradients[i].length_squared());

    if w_sum <= f32::EPSILON {
        return 0.0;
    }

    let tilde_compliance = compliance / (dt * dt);

    (-c - tilde_compliance * lagrange) / (w_sum + tilde_compliance)
}
