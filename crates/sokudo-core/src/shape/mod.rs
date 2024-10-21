use cuboid::CuboidShape;
use glam::Vec3;
use sokudo_io::read::collider::ParsedShape;

pub mod cuboid;

pub trait AbstractShape {
    const GRADIENT_EPSILON: f32 = 0.01;

    /// The signed distance from the given `point` to this [`Shape`], where negative values 
    /// reside inside the object and values of zero are on the surface.
    fn sd(&self, point: Vec3) -> f32;

    /// The gradient of the signed distance field of this [`Shape`].
    ///
    /// Default implementation computes the gradient via finite difference.
    fn sd_gradient(&self, point: Vec3) -> Vec3 {
        let sd_p = self.sd(point);

        let d_sd_dx = (self.sd(Vec3::new(point.x + Self::GRADIENT_EPSILON, point.y, point.z)) - sd_p)
            / Self::GRADIENT_EPSILON;
        let d_sd_dy = (self.sd(Vec3::new(point.x, point.y + Self::GRADIENT_EPSILON, point.z)) - sd_p)
            / Self::GRADIENT_EPSILON;
        let d_sd_dz = (self.sd(Vec3::new(point.x, point.y, point.z + Self::GRADIENT_EPSILON)) - sd_p)
            / Self::GRADIENT_EPSILON;

        Vec3::new(d_sd_dx, d_sd_dy, d_sd_dz)
    }

    /// Projects the given `point` onto the surface of this [`Shape`].
    fn exterior_point(&self, point: Vec3) -> Vec3 {
        point - self.sd_gradient(point) * self.sd(point)
    }

    /// Tests whether or not the given `point` intersects this [`Shape`].
    fn intersects(&self, point: Vec3) -> bool {
        self.sd(point) <= 0.0
    }
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
}

impl From<ParsedShape> for Shape {
    fn from(value: ParsedShape) -> Self {
        match value {
            ParsedShape::Cuboid => Shape::Cuboid(CuboidShape),
        }
    }
}
