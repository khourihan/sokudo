use glam::{Quat, Vec3};
use parry3d::query::PersistentQueryDispatcher;

use super::{contact::{ContactData, ContactManifold, PackedFeatureId, PointContact, SingleContact}, rigid_body::RigidBody, util::FromNaType};

pub type UnsupportedShape = parry3d::query::Unsupported;

pub fn contact(
    collider1: &RigidBody,
    position1: Vec3,
    rotation1: Quat,
    collider2: &RigidBody,
    position2: Vec3,
    rotation2: Quat,
    prediction_distance: f32,
) -> Result<Option<SingleContact>, UnsupportedShape> {
    let isometry1 = isometry(position1, rotation1);
    let isometry2 = isometry(position2, rotation2);

    parry3d::query::contact(
        &isometry1,
        collider1.scaled_shape.0.as_ref(),
        &isometry2,
        collider2.scaled_shape.0.as_ref(),
        prediction_distance,
    )
    .map(|contact| {
        if let Some(contact) = contact {
            let point1 = rotation1.inverse() * Vec3::from_na(contact.point1);
            let point2 = rotation2.inverse() * Vec3::from_na(contact.point2);
            let normal1 = (rotation1.inverse() * Vec3::from_na(contact.normal1)).normalize();
            let normal2 = (rotation2.inverse() * Vec3::from_na(contact.normal2)).normalize();

            if !normal1.is_normalized() || !normal2.is_normalized() {
                return None;
            }

            Some(SingleContact {
                point1,
                point2,
                normal1,
                normal2,
                depth: -contact.dist,
            })
        } else {
            None
        }
    })
}

pub fn contact_manifolds(
    collider1: &RigidBody,
    position1: Vec3,
    rotation1: Quat,
    collider2: &RigidBody,
    position2: Vec3,
    rotation2: Quat,
    prediction_distance: f32,
) -> Vec<ContactManifold> {
    let isometry1 = isometry(position1, rotation1);
    let isometry2 = isometry(position2, rotation2);
    let isometry12 = isometry1.inv_mul(&isometry2);

    // TODO: reuse from previous frame
    let mut manifolds: Vec<parry3d::query::ContactManifold<(), ()>> = vec![];

    let result = parry3d::query::DefaultQueryDispatcher.contact_manifolds(
        &isometry12,
        collider1.scaled_shape.0.as_ref(),
        collider2.scaled_shape.0.as_ref(),
        prediction_distance,
        &mut manifolds,
        &mut None,
    );

    if result.is_err() {
        if let (Some(shape1), Some(shape2)) = (
            collider1.scaled_shape.as_support_map(),
            collider2.scaled_shape.as_support_map(),
        ) {
            if let Some(contact) = parry3d::query::contact::contact_support_map_support_map(
                &isometry12,
                shape1,
                shape2,
                prediction_distance,
            ) {
                let normal1 = Vec3::from_na(contact.normal1);
                let normal2 = Vec3::from_na(contact.normal2);

                if !normal1.is_normalized() || !normal2.is_normalized() {
                    return vec![];
                }

                return vec![ContactManifold {
                    normal1,
                    normal2,
                    contacts: vec![ContactData {
                        point1: Vec3::from_na(contact.point1),
                        point2: Vec3::from_na(contact.point2),
                        normal1,
                        normal2,
                        depth: -contact.dist,
                        feature_id1: PackedFeatureId::UNKNOWN,
                        feature_id2: PackedFeatureId::UNKNOWN,
                    }],
                    index: 0,
                }];
            }
        }
    }

    let mut manifold_index = 0;

    manifolds
        .iter()
        .filter_map(|manifold| {
            let subpos1 = manifold.subshape_pos1.unwrap_or_default();
            let subpos2 = manifold.subshape_pos2.unwrap_or_default();
            let normal1 = Vec3::from_na(subpos1
                .rotation
                .transform_vector(&manifold.local_n1)
                .normalize());
            let normal2 = Vec3::from_na(subpos2
                .rotation
                .transform_vector(&manifold.local_n2)
                .normalize());

            if !normal1.is_normalized() || !normal2.is_normalized() {
                return None;
            }

            let manifold = ContactManifold {
                normal1,
                normal2,
                contacts: manifold
                    .contacts()
                    .iter()
                    .filter_map(|contact| {
                        // TODO: margin
                        if contact.dist > 0.0 {
                            return None;
                        }

                        Some(ContactData {
                            point1: Vec3::from_na(subpos1.transform_point(&contact.local_p1)),
                            point2: Vec3::from_na(subpos2.transform_point(&contact.local_p2)),
                            normal1,
                            normal2,
                            depth: -contact.dist,
                            feature_id1: contact.fid1.into(),
                            feature_id2: contact.fid2.into(),
                        })
                    })
                    .collect(),
                index: manifold_index,
            };

            manifold_index += 1;

            Some(manifold)
        })
        .collect()
}

pub fn contact_point(
    collider1: &RigidBody,
    position1: Vec3,
    rotation1: Quat,
    point: Vec3,
) -> Option<PointContact> {
    let isometry1 = isometry(position1, rotation1);

    let (proj, feature_id) = collider1.scaled_shape.project_point_and_get_feature(
        &isometry1,
        &parry3d::math::Point::<f32>::from_na(point),
    );

    if !proj.is_inside {
        return None;
    }

    let n = Vec3::from_na(proj.point) - point;
    let depth = n.length();
    let normal = if depth.abs() <= 2e-4 { Vec3::ZERO } else { n / depth };

    Some(PointContact {
        point,
        normal,
        depth,
        feature_id: feature_id.into(),
    })
}

fn isometry(position: Vec3, rotation: Quat) -> parry3d::math::Isometry<f32> {
    let r = rotation.to_scaled_axis();
    parry3d::math::Isometry::<f32>::new(
        parry3d::math::Vector::<f32>::new(position.x, position.y, position.z),
        parry3d::math::Vector::<f32>::new(r.x, r.y, r.z),
    )
}
