use glam::{EulerRot, Quat, Vec3};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
#[serde(rename = "Rotation")]
pub enum RawRotation {
    Quat {
        x: f32,
        y: f32,
        z: f32,
        w: f32,
    },
    EulerAngles {
        yaw: f32,
        pitch: f32,
        roll: f32,
    },
    AxisAngle {
        axis: Vec3,
        angle: f32,
    },
}

impl Default for RawRotation {
    fn default() -> Self {
        Self::Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

impl From<RawRotation> for Quat {
    fn from(val: RawRotation) -> Self {
        match val {
            RawRotation::Quat { x, y, z, w } => Quat::from_xyzw(x, y, z, w),
            RawRotation::EulerAngles { yaw, pitch, roll } => Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll),
            RawRotation::AxisAngle { axis, angle } => Quat::from_axis_angle(axis, angle),
        }
    }
}
