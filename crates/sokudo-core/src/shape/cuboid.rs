use glam::{UVec3, Vec2, Vec3, Vec3Swizzles};

use super::AbstractShape;

/// A cube shape with unit side lengths centered at the origin.
#[derive(Debug)]
pub struct CuboidShape;

impl AbstractShape for CuboidShape {
    fn sd(&self, point: Vec3) -> f32 {
        let q = point.abs() - 0.5;
        q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
    }

    // fn sd_gradient(&self, point: Vec3) -> Vec3 {
    //     let w = point.abs() - 0.5;
    //     let s = point.signum();
    //
    //     let g = w.max_element();
    //     let q = w.max(Vec3::ZERO);
    //     let l = q.length();
    //
    //     s * (if g > 0.0 {
    //         q / l
    //     } else if w.x > w.y && w.x > w.z {
    //         Vec3::X
    //     } else if w.y > w.z {
    //         Vec3::Y
    //     } else {
    //         Vec3::Z
    //     })
    // }

    fn vertices(&self, resolution: UVec3) -> Vec<Vec3> {
        let mut vertices = Vec::new();

        for (up, x, y) in
            [(Vec3::Y, 0, 2), (Vec3::NEG_Y, 0, 2), (Vec3::X, 2, 1), (Vec3::NEG_X, 2, 1), (Vec3::Z, 1, 0), (Vec3::NEG_Z, 1, 0)]
        {
            let right = up.yzx();
            let front = up.cross(right);

            for i in 0..=resolution[x] {
                for j in 0..=resolution[y] {
                    let uv = Vec2::new(i as f32, j as f32)
                        / Vec2::new(resolution[x] as f32, resolution[y] as f32);

                    let p = up * 0.5 + (uv.x - 0.5) * right + (uv.y - 0.5) * front;

                    if !vertices.contains(&p) {
                        vertices.push(p);
                    }
                }
            }
        }

        vertices
    }

    fn moments(&self, scale: Vec3) -> Vec3 {
        scale.normalize()
    }
}
