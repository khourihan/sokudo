use glam::UVec3;

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
    pub const fn material_roughness() -> f32 {
        1.0
    }

    #[inline(always)]
    pub const fn material_resilience() -> f32 {
        0.2
    }

    #[inline(always)]
    pub const fn material_hardness() -> f32 {
        1.0
    }
}
