use glam::{Affine3A, Mat3, Mat4, Quat, Vec3};
use sokudo_io::{read::transform::ParsedTransform, write::transform::WriteTransform};

/// Describes the position of an object.
#[derive(Debug, Clone)]
pub struct Transform {
    /// Position of the object, in meters.
    pub translate: Vec3,
    /// Rotation of the object.
    pub rotate: Quat,
    /// Scale of the object, in meters.
    pub scale: Vec3,
}

impl Transform {
    /// An identity [`Transform`] with no translation, no rotation, and a scale of 1 on all axes.
    pub const IDENTITY: Self = Transform {
        translate: Vec3::ZERO,
        rotate: Quat::IDENTITY,
        scale: Vec3::ONE,
    };

    /// Creates a new [`Transform`] at the position `(x, y, z)`. Rotation will be 0 and scale 1 on
    /// all axes.
    #[inline]
    pub const fn from_xyz(x: f32, y: f32, z: f32) -> Self {
        Self::from_translation(Vec3::new(x, y, z))
    }

    /// Extracts the translation, rotation, and scale of a matrix.
    #[inline]
    pub fn from_matrix(world_from_local: Mat4) -> Self {
        let (scale, rotate, translate) = world_from_local.to_scale_rotation_translation();

        Transform {
            translate,
            rotate,
            scale,
        }
    }

    /// Creates a new [`Transform`], given its `translation`. Rotation will be 0 and scale 1 on
    /// all axes.
    #[inline]
    pub const fn from_translation(translate: Vec3) -> Self {
        Self {
            translate,
            ..Self::IDENTITY
        }
    }

    /// Creates a new [`Transform`], given its `rotation`. Translation will be 0 and scale 1 on
    /// all axes.
    #[inline]
    pub const fn from_rotation(rotate: Quat) -> Self {
        Transform {
            rotate,
            ..Self::IDENTITY
        }
    }

    /// Creates a new [`Transform`], given its `scale`. Translation will be 0 and rotation 0 on
    /// all axes.
    #[inline]
    pub const fn from_scale(scale: Vec3) -> Self {
        Transform {
            scale,
            ..Self::IDENTITY
        }
    }

    /// Returns this [`Transform`] with a new rotation so that [`Transform::forward`]
    /// points towards the `target` position and [`Transform::up`] points towards `up`.
    #[inline]
    #[must_use]
    pub fn looking_at(mut self, target: Vec3, up: Vec3) -> Self {
        self.look_at(target, up);
        self
    }

    /// Returns this [`Transform`] with a new rotation so that [`Transform::forward`]
    /// points in the given `direction` and [`Transform::up`] points towards `up`.
    #[inline]
    #[must_use]
    pub fn looking_to(mut self, direction: Vec3, up: Vec3) -> Self {
        self.look_to(direction, up);
        self
    }

    /// Returns the 3d affine transformation matrix from this transforms translation,
    /// rotation, and scale.
    #[inline]
    pub fn compute_matrix(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(self.scale, self.rotate, self.translate)
    }

    /// Returns the 3d affine transformation matrix from this transforms translation,
    /// rotation, and scale.
    #[inline]
    pub fn compute_affine(&self) -> Affine3A {
        Affine3A::from_scale_rotation_translation(self.scale, self.rotate, self.translate)
    }

    /// Get the unit vector in the local `X` direction.
    #[inline]
    pub fn local_x(&self) -> Vec3 {
        self.rotate * Vec3::X
    }

    /// Equivalent to [`-local_x()`][Transform::local_x()]
    #[inline]
    pub fn left(&self) -> Vec3 {
        -self.local_x()
    }

    /// Equivalent to [`local_x()`][Transform::local_x()]
    #[inline]
    pub fn right(&self) -> Vec3 {
        self.local_x()
    }

    /// Get the unit vector in the local `Y` direction.
    #[inline]
    pub fn local_y(&self) -> Vec3 {
        self.rotate * Vec3::Y
    }

    /// Equivalent to [`local_y()`][Transform::local_y]
    #[inline]
    pub fn up(&self) -> Vec3 {
        self.local_y()
    }

    /// Equivalent to [`-local_y()`][Transform::local_y]
    #[inline]
    pub fn down(&self) -> Vec3 {
        -self.local_y()
    }

    /// Get the unit vector in the local `Z` direction.
    #[inline]
    pub fn local_z(&self) -> Vec3 {
        self.rotate * Vec3::Z
    }

    /// Equivalent to [`-local_z()`][Transform::local_z]
    #[inline]
    pub fn forward(&self) -> Vec3 {
        -self.local_z()
    }

    /// Equivalent to [`local_z()`][Transform::local_z]
    #[inline]
    pub fn back(&self) -> Vec3 {
        self.local_z()
    }

    /// Rotates this [`Transform`] by the given rotation.
    #[inline]
    pub fn rotate(&mut self, rotation: Quat) {
        self.rotate = rotation * self.rotate;
    }

    /// Rotates this [`Transform`] by the given `rotation`.
    ///
    /// The `rotation` is relative to this [`Transform`]'s current rotation.
    #[inline]
    pub fn rotate_local(&mut self, rotation: Quat) {
        self.rotate *= rotation;
    }

    /// Rotates this [`Transform`] so that [`Transform::forward`] points towards the `target` position,
    /// and [`Transform::up`] points towards `up`.
    #[inline]
    pub fn look_at(&mut self, target: Vec3, up: Vec3) {
        self.look_to(target - self.translate, up);
    }

    /// Rotates this [`Transform`] so that [`Transform::forward`] points in the given `direction`
    /// and [`Transform::up`] points towards `up`.
    #[inline]
    pub fn look_to(&mut self, direction: Vec3, up: Vec3) {
        let back = -direction;
        let right = up.cross(back).try_normalize().unwrap_or_else(|| up.any_orthonormal_vector());
        let up = back.cross(right);
        self.rotate = Quat::from_mat3(&Mat3::from_cols(right, up, back));
    }

    /// Transforms the given `point` from local coordinates to global coordinates.
    /// This applies scale, rotation, and translation.
    #[inline]
    pub fn globalize(&self, point: Vec3) -> Vec3 {
        (self.rotate * (self.scale * point)) + self.translate
    }

    /// Transforms the given `point` from global coordinates to local coordinates.
    /// This inversely applies scale, rotation, and translation.
    #[inline]
    pub fn localize(&self, point: Vec3) -> Vec3 {
        (self.rotate.inverse() * (point - self.translate)) / self.scale
    }

    /// Transforms the given `direction` from local coordinates to global coordinates.
    /// This applies just rotation.
    #[inline]
    pub fn globalize_direction(&self, direction: Vec3) -> Vec3 {
        self.rotate * direction
    }

    /// Transforms the given `direction` from global coordinates to local coordinates.
    /// This inversely just rotation.
    #[inline]
    pub fn localize_direction(&self, direction: Vec3) -> Vec3 {
        self.rotate.inverse() * direction
    }

    /// Computes the inverse of this [`Transform`].
    #[inline]
    pub fn inverse(&self) -> Transform {
        Transform {
            translate: -self.translate,
            rotate: self.rotate.inverse(),
            scale: self.scale.recip(),
        }
    }
}

impl From<ParsedTransform> for Transform {
    fn from(value: ParsedTransform) -> Self {
        Transform {
            translate: value.translate,
            rotate: value.rotate,
            scale: value.scale,
        }
    }
}

impl From<&Transform> for WriteTransform {
    fn from(value: &Transform) -> Self {
        WriteTransform {
            translate: value.translate,
            rotate: value.rotate,
        }
    }
}
