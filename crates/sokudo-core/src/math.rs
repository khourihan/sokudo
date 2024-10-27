use glam::{Mat3, Vec3};

/// Computes the skew-symmetric matrix corresponding to the given vector.
///
/// ```text
///                          [   0  -v.z  v.y ]
/// skew_symmetric_mat3(v) = [  v.z   0  -v.x ]
///                          [ -v.y  v.x   0  ]
/// ```
pub fn skew_symmetric_mat3(v: Vec3) -> Mat3 {
    Mat3::from_cols_array(&[
        0.0, v.z, -v.y,
        -v.z, 0.0, v.x,
        v.y, -v.x, 0.0,
    ])
}
