use glam::Vec3;

pub struct DefaultOptions;

impl DefaultOptions {
    #[inline(always)]
    pub const fn dt() -> f32 {
        0.02
    }

    pub const fn gravity() -> Vec3 {
        Vec3::ZERO
    }

    pub const fn substeps() -> u32 {
        4
    }

    pub const fn constraint_iterations() -> u32 {
        10
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

    #[inline(always)]
    pub const fn material_dynamic_friction() -> f32 {
        1.0
    }

    #[inline(always)]
    pub const fn material_stiffness() -> f32 {
        1.0
    }
}
