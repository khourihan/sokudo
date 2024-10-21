use glam::{EulerRot, Quat, Vec3};
use serde::{de::{self, MapAccess, Visitor}, Deserialize, Deserializer};

#[derive(Debug)]
pub struct ParsedTransform {
    pub translate: Vec3,
    pub rotate: Quat,
    pub scale: Vec3,
}

impl Default for ParsedTransform {
    fn default() -> Self {
        Self {
            translate: Vec3::ZERO,
            rotate: Quat::IDENTITY,
            scale: Vec3::ONE,
        }
    }
}

impl<'de> Deserialize<'de> for ParsedTransform {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>
    {
        deserializer.deserialize_struct("Transform", FIELDS, TransformVisitor)
    }
}

struct TransformVisitor;

impl<'de> Visitor<'de> for TransformVisitor {
    type Value = ParsedTransform;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        formatter.write_str("struct `Transform`")
    }

    fn visit_map<A>(self, mut map: A) -> Result<Self::Value, A::Error>
    where
        A: MapAccess<'de>
    {
        let mut translate: Option<Vec3> = None;
        let mut rotate: Option<Rotation> = None;
        let mut scale: Option<Vec3> = None;

        while let Some(key) = map.next_key()? {
            match key {
                Field::Translate => {
                    if translate.is_some() {
                        return Err(de::Error::duplicate_field("translate"));
                    }
                    translate = Some(map.next_value()?);
                },
                Field::Rotate => {
                    if rotate.is_some() {
                        return Err(de::Error::duplicate_field("rotate"));
                    }
                    rotate = Some(map.next_value()?);
                },
                Field::Scale => {
                    if scale.is_some() {
                        return Err(de::Error::duplicate_field("scale"));
                    }
                    scale = Some(map.next_value()?);
                },
            }
        }

        Ok(ParsedTransform {
            translate: translate.unwrap_or(Vec3::ZERO),
            rotate: rotate.map(Quat::from).unwrap_or(Quat::IDENTITY),
            scale: scale.unwrap_or(Vec3::ONE),
        })
    }
}

enum Field {
    Translate,
    Rotate,
    Scale,
}

const FIELDS: &[&str] = &["translate", "rotate", "scale"];

impl<'de> Deserialize<'de> for Field {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>
    {
        deserializer.deserialize_identifier(FieldVisitor)
    }
}

struct FieldVisitor;

impl<'de> Visitor<'de> for FieldVisitor {
    type Value = Field;

    fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
        formatter.write_str("`translate`, `rotate`, or `scale`")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: de::Error
    {
        match v {
            "translate" => Ok(Field::Translate),
            "rotate" => Ok(Field::Rotate),
            "scale" => Ok(Field::Scale),
            _ => Err(de::Error::unknown_field(v, FIELDS)),
        }
    }
}

#[derive(Deserialize)]
enum Rotation {
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

impl From<Rotation> for Quat {
    fn from(val: Rotation) -> Self {
        match val {
            Rotation::Quat { x, y, z, w } => Quat::from_xyzw(x, y, z, w),
            Rotation::EulerAngles { yaw, pitch, roll } => Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll),
            Rotation::AxisAngle { axis, angle } => Quat::from_axis_angle(axis, angle),
        }
    }
}
