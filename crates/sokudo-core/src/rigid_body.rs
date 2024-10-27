use glam::{Mat3, UVec3, Vec3};
use sokudo_io::read::collider::ParsedRigidBody;

use crate::{shape::{AbstractShape, Shape}, transform::Transform};

#[derive(Debug)]
pub struct RigidBody {
    /// The transform of this rigid body. 
    ///
    /// Note that the body's translation is equal to its center of mass.
    /// This means that, in local space, the body's center of mass should be at the origin.
    pub transform: Transform,
    /// The shape of this rigid body.
    pub shape: Shape,
    /// The mass of this rigid body.
    pub mass: f32,
    /// The resolution of the vertices, in all three dimensions.
    pub vertex_resolution: UVec3,
    /// The object's moments of inertia, represented as the three principle axes.
    pub moments: Vec3,
    /// The precomputed vertices to test for intersections on this rigid body.
    pub vertices: Vec<Vec3>,
}

impl RigidBody {
    pub fn compute_vertices(&mut self) {
        if self.vertices.is_empty() {
            self.vertices = self.shape.vertices(self.vertex_resolution);
        }
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

    pub fn compute_moments(&mut self) {
        self.moments = self.shape.moments(self.transform.scale);
    }

    pub fn inverse_inertia_tensor(&self, r: Mat3) -> Mat3 {
        let mut rt = r.transpose();
        let moments = self.moments * self.mass;

        rt.x_axis /= moments.x;
        rt.y_axis /= moments.y;
        rt.z_axis /= moments.z;

        r * rt
    }
}

impl From<ParsedRigidBody> for RigidBody {
    fn from(value: ParsedRigidBody) -> Self {
        RigidBody {
            transform: value.transform.into(),
            shape: value.shape.into(),
            mass: value.mass,
            vertex_resolution: if value.vertex_resolution == UVec3::ZERO {
                UVec3::ONE
            } else {
                value.vertex_resolution
            },
            moments: Vec3::ZERO,
            vertices: value.vertices,
        }
    }
}
