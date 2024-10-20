use glam::Vec3;

use super::AbstractShape;

/// A cube shape with unit side lengths.
#[derive(Debug)]
pub struct CuboidShape;

impl AbstractShape for CuboidShape {
    fn sd(&self, p: Vec3) -> f32 {
        let q = p.abs() - 0.5;
        q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
    }

    fn sd_gradient(&self, p: Vec3) -> Vec3 {
        let w = p.abs() - 0.5;
        let s = p.signum();

        let g = w.max_element();
        let q = w.max(Vec3::ZERO);
        let l = q.length();

        s * (if g > 0.0 {
            q / l
        } else if w.x > w.y && w.x > w.z {
            Vec3::X
        } else if w.y > w.z {
            Vec3::Y
        } else {
            Vec3::Z
        })
    }
}
