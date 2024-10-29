use glam::{UVec3, Vec3};

pub struct DefaultOptions;

impl DefaultOptions {
    #[inline(always)]
    pub const fn vertex_resolution() -> UVec3 {
        UVec3::ONE
    }

    #[inline(always)]
    pub const fn mass() -> f32 {
        1.0
    }

    #[inline(always)]
    pub const fn density() -> f32 {
        1.0
    }

    #[inline(always)]
    pub const fn scale() -> Vec3 {
        Vec3::ONE
    }

    #[inline(always)]
    pub const fn material_restitution() -> f32 {
        1.0
    }
}
