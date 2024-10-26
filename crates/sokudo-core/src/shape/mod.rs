use cuboid::CuboidShape;
use glam::{UVec3, Vec3};
use sokudo_io::read::collider::ParsedShape;

pub mod cuboid;

pub trait AbstractShape {
    const GRADIENT_EPSILON: f32 = 0.01;

    /// The signed distance from the given `point` to this [`Shape`], where negative values 
    /// reside inside the object and values of zero are on the surface.
    fn sd(&self, point: Vec3) -> f32;

    /// The gradient of the signed distance field of this [`Shape`] at the given `point`.
    /// Note that this always points towards the spine of the SDF, or the minimum (negative) values
    /// of it.
    ///
    /// Default implementation computes the gradient via finite difference.
    fn sd_gradient(&self, point: Vec3) -> Vec3 {
        let d_sd_dx = (
            self.sd(Vec3::new(point.x + Self::GRADIENT_EPSILON, point.y, point.z))
                - self.sd(Vec3::new(point.x - Self::GRADIENT_EPSILON, point.y, point.z))
        ) / (2.0 * Self::GRADIENT_EPSILON);
        let d_sd_dy = (
            self.sd(Vec3::new(point.x, point.y + Self::GRADIENT_EPSILON, point.z))
                 - self.sd(Vec3::new(point.x, point.y - Self::GRADIENT_EPSILON, point.z))
        ) / (2.0 * Self::GRADIENT_EPSILON);
        let d_sd_dz = (
            self.sd(Vec3::new(point.x, point.y, point.z + Self::GRADIENT_EPSILON))
                - self.sd(Vec3::new(point.x, point.y, point.z - Self::GRADIENT_EPSILON))
        ) / (2.0 * Self::GRADIENT_EPSILON);

        Vec3::new(d_sd_dx, d_sd_dy, d_sd_dz).normalize_or_zero()
    }

    fn vertices(&self, resolution: UVec3) -> Vec<Vec3>;

    fn moments(&self, scale: Vec3) -> Vec3;
}

#[derive(Debug)]
pub enum Shape {
    Cuboid(CuboidShape),
}

impl AbstractShape for Shape {
    fn sd(&self, point: Vec3) -> f32 {
        match self {
            Shape::Cuboid(c) => c.sd(point),
        }
    }

    fn sd_gradient(&self, point: Vec3) -> Vec3 {
        match self {
            Shape::Cuboid(c) => c.sd_gradient(point),
        }
    }

    fn vertices(&self, resolution: UVec3) -> Vec<Vec3> {
        match self {
            Shape::Cuboid(c) => c.vertices(resolution),
        }
    }

    fn moments(&self, scale: Vec3) -> Vec3 {
        match self {
            Shape::Cuboid(c) => c.moments(scale),
        }
    }
}

impl From<ParsedShape> for Shape {
    fn from(value: ParsedShape) -> Self {
        match value {
            ParsedShape::Cuboid => Shape::Cuboid(CuboidShape),
        }
    }
}
