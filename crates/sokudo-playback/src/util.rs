use bevy::prelude::{Quat as BevyQuat, Vec2 as BevyVec2, Vec3 as BevyVec3, Vec4 as BevyVec4};

use glam::{Quat, Vec2, Vec3, Vec3A, Vec4};

pub trait ToBevyType {
    type Output;

    fn to_bevy(self) -> Self::Output;
}

macro_rules! impl_to_bevy {
    ($ty:ty: $bevy:ty, |$self:ident| => $f:block) => {
        impl ToBevyType for $ty {
            type Output = $bevy;

            fn to_bevy($self) -> Self::Output $f
        }
    }
}

impl_to_bevy!(
    Vec2: BevyVec2,
    |self| => {
        BevyVec2::new(self.x, self.y)
    }
);

impl_to_bevy!(
    Vec3: BevyVec3,
    |self| => {
        BevyVec3::new(self.x, self.y, self.z)
    }
);

impl_to_bevy!(
    Vec3A: BevyVec3,
    |self| => {
        BevyVec3::new(self.x, self.y, self.z)
    }
);

impl_to_bevy!(
    Vec4: BevyVec4,
    |self| => {
        BevyVec4::new(self.x, self.y, self.z, self.w)
    }
);

impl_to_bevy!(
    Quat: BevyQuat,
    |self| => {
        BevyQuat::from_xyzw(self.x, self.y, self.z, self.w)
    }
);
