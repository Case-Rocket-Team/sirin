use nalgebra::{Quaternion, UnitQuaternion};

use crate::state::{Mat3, Vec3};

pub fn skew(vector: &Vec3) -> Mat3 {
    Mat3::new(
        0.0,
        -vector.z,
        vector.y,
        vector.z,
        0.0,
        -vector.x,
        -vector.y,
        vector.x,
        0.0,
    )
}

/// SO(3) exponential represented as a unit quaternion.
pub fn exp_quaternion(rotation_vector: &Vec3) -> UnitQuaternion<f32> {
    let angle_squared = rotation_vector.norm_squared();
    if angle_squared < 1.0e-12 {
        let angle_fourth = angle_squared * angle_squared;
        let scalar = 1.0 - angle_squared / 8.0 + angle_fourth / 384.0;
        let scale = 0.5 - angle_squared / 48.0 + angle_fourth / 3840.0;
        return UnitQuaternion::new_normalize(Quaternion::from_parts(
            scalar,
            rotation_vector * scale,
        ));
    }

    UnitQuaternion::from_scaled_axis(*rotation_vector)
}

pub fn all_finite3(vector: &Vec3) -> bool {
    vector.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn skew_matches_cross_product() {
        let a = Vec3::new(1.0, -2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, -6.0);
        assert!((skew(&a) * b - a.cross(&b)).norm() < 1.0e-6);
    }

    #[test]
    fn exp_handles_zero_and_small_angles() {
        let zero = exp_quaternion(&Vec3::zeros());
        assert!((zero.angle()).abs() < 1.0e-7);

        let rotation = Vec3::new(1.0e-7, -2.0e-7, 3.0e-7);
        let recovered = exp_quaternion(&rotation).scaled_axis();
        assert!((rotation - recovered).norm() < 1.0e-7);
    }
}
