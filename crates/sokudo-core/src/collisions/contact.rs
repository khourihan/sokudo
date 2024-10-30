use glam::Vec3;

use super::collider::ColliderId;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SingleContact {
    /// Contact point in local space of the first body.
    pub point1: Vec3,
    /// Contact point in local space of the second body.
    pub point2: Vec3,
    /// Contact normal expressed in the local space of the first body.
    pub normal1: Vec3,
    /// Contact normal expressed in the local space of the second body.
    pub normal2: Vec3,
    /// Penetration depth.
    pub depth: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PointContact {
    /// Contact point in global space.
    pub point: Vec3,
    /// Contact normal in global space.
    pub normal: Vec3,
    /// Penetration depth.
    pub depth: f32,
    pub feature_id: PackedFeatureId,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactData {
    /// Contact point in local space of the first body.
    pub point1: Vec3,
    /// Contact point in local space of the second body.
    pub point2: Vec3,
    /// Contact normal expressed in the local space of the first body.
    pub normal1: Vec3,
    /// Contact normal expressed in the local space of the second body.
    pub normal2: Vec3,
    /// Penetration depth.
    pub depth: f32,
    pub feature_id1: PackedFeatureId,
    pub feature_id2: PackedFeatureId,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Contacts {
    /// First body in the contact.
    pub collider1: ColliderId,
    /// Second body in the contact.
    pub collider2: ColliderId,
    /// A list of contact manifolds between two colliders.
    /// Each manifold contains one of more contact points, but each contact in a given manifold
    /// shares the same contact normal.
    pub manifolds: Vec<ContactManifold>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ContactManifold {
    /// The contacts in this manifold.
    pub contacts: Vec<ContactData>,
    /// Contact normal shared by all contacts in the manifold, expressed in the local space of the
    /// first body.
    pub normal1: Vec3,
    /// Contact normal shared by all contacts in the manifold, expressed in the local space of the
    /// second body.
    pub normal2: Vec3,
    /// The index of the manifold in the collision.
    pub index: usize,
}

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub struct PackedFeatureId(pub u32);

impl PackedFeatureId {
    pub const UNKNOWN: Self = Self(0);

    const CODE_MASK: u32 = 0x3fff_ffff;
    const HEADER_MASK: u32 = !Self::CODE_MASK;
    const HEADER_VERTEX: u32 = 0b01 << 30;
    const HEADER_EDGE: u32 = 0b10 << 30;
    const HEADER_FACE: u32 = 0b11 << 30;

    #[inline]
    pub fn vertex(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_VERTEX | code)
    }

    #[inline]
    pub fn edge(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_EDGE | code)
    }

    #[inline]
    pub fn face(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_FACE | code)
    }

    #[inline]
    pub fn is_face(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_FACE
    }

    #[inline]
    pub fn is_vertex(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_VERTEX
    }

    #[inline]
    pub fn is_edge(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_EDGE
    }

    #[inline]
    pub fn is_unknown(self) -> bool {
        self == Self::UNKNOWN
    }
}

impl From<u32> for PackedFeatureId {
    #[inline]
    fn from(code: u32) -> Self {
        Self(code)
    }
}

impl From<parry3d::shape::PackedFeatureId> for PackedFeatureId {
    #[inline]
    fn from(value: parry3d::shape::PackedFeatureId) -> Self {
        Self(value.0)
    }
}

impl From<parry3d::shape::FeatureId> for PackedFeatureId {
    #[inline]
    fn from(value: parry3d::shape::FeatureId) -> Self {
        Self(parry3d::shape::PackedFeatureId::from(value).0)
    }
}
