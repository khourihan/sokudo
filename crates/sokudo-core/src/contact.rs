use glam::Vec3;

#[derive(Clone, Debug, PartialEq)]
pub struct RigidBodyContact {
    /// Contact point in the first entity's local coordinates.
    pub point1: Vec3,
    /// Contact point in the second entity's local coordinates.
    pub point2: Vec3,
    /// Contact normal in the first entity's local coordinates.
    pub normal1: Vec3,
    /// Contact normal in the second entity's local coordinates.
    pub normal2: Vec3,
    /// Penetration depth.
    pub depth: f32,
}
