use glam::{Mat3, Quat, Vec3};
use parry3d::shape::SharedShape;
use sokudo_io::read::collider::ParsedRigidBody;

#[derive(Debug)]
pub struct RigidBody {
    /// The raw unscaled shape of this rigid body.
    pub shape: SharedShape,
    /// The scaled version of the shape of this rigid body.
    pub scaled_shape: SharedShape,
    /// The scale of the rigid body, in all three dimensions.
    pub scale: Vec3,

    /// The inverse mass of this rigid body.
    pub inverse_mass: f32,
    /// The inverse of the inertia tensor of this rigid body, in local coordinates.
    pub inertia_tensor: InertiaTensor, 
    /// The center of mass of this rigid body, in local coordinates.
    pub center_of_mass: Vec3,
    /// The uniform density of this rigid body.
    pub density: f32,

    pub rotation: Quat,
    pub previous_rotation: Quat,
    pub angular_velocity: Vec3,
    pub previous_angular_velocity: Vec3,

    /// Damping on the rigid body's angular velocity.
    pub angular_damping: f32,
}

impl RigidBody {
    pub fn compute_mass_properties(&mut self) {
        let props = self.scaled_shape.mass_properties(self.density);

        self.inverse_mass = props.inv_mass;
        self.inertia_tensor = props.reconstruct_inverse_inertia_matrix().into();
        self.center_of_mass = Vec3::new(props.local_com.x, props.local_com.y, props.local_com.z);
    }

    // TODO: Maybe store global inverse inertia tensor as well + update per frame?
    #[inline]
    pub fn global_inverse_inertia(&self) -> Mat3 {
        self.inertia_tensor.rotate(self.rotation).inverse()
    }

    /// Compute the generalized inverse mass of this rigid body at point `r` when applying
    /// positional correction along the vector `n`, where `r` is relative to the body's center of
    /// mass in global coordinates.
    #[inline]
    pub fn positional_inverse_mass(&self, r: Vec3, n: Vec3) -> f32 {
        let r_cross_n = r.cross(n);
        self.inverse_mass + r_cross_n.dot(self.global_inverse_inertia() * r_cross_n)
    }
}

impl From<ParsedRigidBody> for RigidBody {
    fn from(value: ParsedRigidBody) -> Self {
        let (unscaled, scaled) = value.shape.to_scaled_shape(value.scale, 0);

        RigidBody {
            shape: unscaled,
            scaled_shape: scaled,
            scale: value.scale,

            inverse_mass: 0.0,
            inertia_tensor: InertiaTensor::default(),
            center_of_mass: Vec3::ZERO,
            density: value.density,

            rotation: value.rotation,
            previous_rotation: value.rotation,
            angular_velocity: value.angular_velocity,
            previous_angular_velocity: value.angular_velocity,

            angular_damping: value.angular_damping,
        }
    }
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct InertiaTensor {
    inverse: Mat3
}

impl Default for InertiaTensor {
    fn default() -> Self {
        Self::INFINITY
    }
}

impl InertiaTensor {
    pub const INFINITY: Self = Self {
        inverse: Mat3::ZERO,
    };

    #[inline]
    pub fn new(principal_moments: Vec3) -> Self {
        let rcp = principal_moments.recip();
        Self::from_inverse_tensor(Mat3::from_diagonal(
            if rcp.is_finite() {
                rcp
            } else {
                Vec3::ZERO
            }
        ))
    }

    #[inline]
    pub fn from_tensor(tensor: Mat3) -> Self {
        Self::from_inverse_tensor(tensor.inverse())
    }

    #[inline]
    pub fn from_inverse_tensor(inverse_tensor: Mat3) -> Self {
        Self {
            inverse: inverse_tensor
        }
    }

    #[inline]
    pub fn inverse(self) -> Mat3 {
        self.inverse
    }

    #[inline]
    pub fn tensor(self) -> Mat3 {
        self.inverse.inverse()
    }

    #[inline]
    pub fn inverse_mut(&mut self) -> &mut Mat3 {
        &mut self.inverse
    }

    #[inline]
    pub fn rotate(self, q: Quat) -> Self {
        let r = Mat3::from_quat(q);
        Self::from_inverse_tensor((r * self.inverse) * r.transpose())
    }

    #[inline]
    pub fn is_finite(&self) -> bool {
        !self.is_infinite() && !self.is_nan()
    }

    #[inline]
    pub fn is_infinite(&self) -> bool {
        *self == Self::INFINITY
    }

    #[inline]
    pub fn is_nan(&self) -> bool {
        self.inverse.is_nan()
    }
}

impl From<parry3d::na::Matrix3<f32>> for InertiaTensor {
    #[inline]
    fn from(value: parry3d::na::Matrix3<f32>) -> Self {
        Self::from_inverse_tensor(Mat3::from_cols(
            Vec3::new(value[(0, 0)], value[(0, 1)], value[(0, 2)]),
            Vec3::new(value[(1, 0)], value[(1, 1)], value[(1, 2)]),
            Vec3::new(value[(2, 0)], value[(2, 1)], value[(2, 2)]),
        ))
    }
}
