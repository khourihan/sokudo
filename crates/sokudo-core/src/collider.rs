use glam::{Mat3, UVec3, Vec3};
use sokudo_io::{read::collider::{ParsedCollider, ParsedForces, ParsedMaterial}, write::{collider::WriteCollider, inspect::InspectElements}};

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

    /// The resolution of the vertices, in all three dimensions.
    pub vertex_resolution: UVec3,
    /// The object's moments of inertia, represented as the three principle axes.
    pub moments: Vec3,

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

    /// The precomputed vertices to test for intersections on this collider.
    pub vertices: Vec<Vec3>,
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
    /// Simulates the collision between two [`Collider`]s, applying the necessary forces to resolve
    /// the collision if necessary.
    pub fn collide(&mut self, other: &mut Self, inspector: &mut InspectElements) {
        let mut vertices = self.vertices.as_ptr();

        for _ in 0..self.vertices.len() {
            let vertex = unsafe { *vertices };
            unsafe {
                vertices = vertices.add(1);
            }

            let vertex = self.transform.globalize(vertex);

            // Ignore if no intersection.
            if other.sd(vertex) > 0.0 {
                continue;
            }

            // Project the vertex onto the closest point on the surface of the other SDF.
            let exterior_point = vertex - other.sd_gradient(vertex) * other.sd(vertex);

            // Find the escape vector of the vertex.
            let d = exterior_point - vertex;

            self.apply_force(vertex, d * 1000.0);
            other.apply_force(vertex, -d * 1000.0);

            inspector.add_point(format!("vertex_{}", vertices as usize), vertex);
        }
    }

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

    pub fn compute_moments(&mut self) {
        self.moments = self.shape.moments(self.transform.scale);
    }

    pub fn inertia_tensor(&self, r: Mat3) -> Mat3 {
        let mut rt = r.transpose();
        let moments = self.moments * self.mass;

        rt.x_axis /= moments.x;
        rt.y_axis /= moments.y;
        rt.z_axis /= moments.z;

        r * rt
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

            vertex_resolution: if value.vertex_resolution == UVec3::ZERO {
                UVec3::ONE
            } else {
                value.vertex_resolution
            },
            moments: Vec3::ZERO,

            forces: value.forces.into(),
            velocity: value.velocity,
            angular_velocity: value.angular_velocity,
            angular_momentum: value.angular_momentum,
            acceleration: value.acceleration,

            vertices: value.vertices,
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
