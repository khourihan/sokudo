use glam::Vec3;

pub trait FromNaType<T> {
    fn from_na(v: T) -> Self;
}

macro_rules! impl_from_na {
    ($($ty:ty: $na:ty),* $(,)* |$v:ident| => $f:block) => {
        $(
            impl FromNaType<$na> for $ty {
                #[inline]
                fn from_na($v: $na) -> Self $f
            }
        )*
    }
}

impl_from_na!(
    Vec3: parry3d::math::Point<f32>,
    Vec3: parry3d::math::Vector<f32>,
    Vec3: parry3d::na::Unit<parry3d::math::Vector<f32>>,

    |v| => {
        Vec3::new(v.x, v.y, v.z)
    }
);

impl_from_na!(
    parry3d::math::Point<f32>: Vec3,
    |v| => {
        parry3d::math::Point::<f32>::new(v.x, v.y, v.z)
    }
);

impl_from_na!(
    parry3d::math::Vector<f32>: Vec3,
    |v| => {
        parry3d::math::Vector::<f32>::new(v.x, v.y, v.z)
    }
);
