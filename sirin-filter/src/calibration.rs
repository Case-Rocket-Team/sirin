use crate::state::{Mat3, Vec3};

#[derive(Debug, Clone)]
pub struct SensorCalibration {
    pub accel_bias_mps2: Vec3,
    pub accel_scale: Mat3,
    pub accel_to_body: Mat3,
    pub gyro_bias_radps: Vec3,
    pub gyro_scale: Mat3,
    pub gyro_to_body: Mat3,
    pub mag_hard_iron_ut: Vec3,
    pub mag_soft_iron: Mat3,
    pub mag_to_body: Mat3,
}

impl Default for SensorCalibration {
    fn default() -> Self {
        Self {
            accel_bias_mps2: Vec3::zeros(),
            accel_scale: Mat3::identity(),
            accel_to_body: Mat3::identity(),
            gyro_bias_radps: Vec3::zeros(),
            gyro_scale: Mat3::identity(),
            gyro_to_body: Mat3::identity(),
            mag_hard_iron_ut: Vec3::zeros(),
            mag_soft_iron: Mat3::identity(),
            mag_to_body: Mat3::identity(),
        }
    }
}

impl SensorCalibration {
    pub fn calibrate_accel(&self, raw_mps2: &Vec3) -> Vec3 {
        self.accel_to_body * (self.accel_scale * (raw_mps2 - self.accel_bias_mps2))
    }

    pub fn calibrate_gyro(&self, raw_radps: &Vec3) -> Vec3 {
        self.gyro_to_body * (self.gyro_scale * (raw_radps - self.gyro_bias_radps))
    }

    pub fn calibrate_mag(&self, raw_ut: &Vec3) -> Vec3 {
        self.mag_to_body * (self.mag_soft_iron * (raw_ut - self.mag_hard_iron_ut))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_calibration_is_a_no_op() {
        let calibration = SensorCalibration::default();
        let value = Vec3::new(1.0, -2.0, 3.0);
        assert_eq!(calibration.calibrate_accel(&value), value);
        assert_eq!(calibration.calibrate_gyro(&value), value);
        assert_eq!(calibration.calibrate_mag(&value), value);
    }

    #[test]
    fn calibration_applies_bias_scale_and_rotation() {
        let mut calibration = SensorCalibration::default();
        calibration.accel_bias_mps2 = Vec3::new(1.0, 0.0, 0.0);
        calibration.accel_scale = Mat3::from_diagonal(&Vec3::new(2.0, 1.0, 1.0));
        calibration.accel_to_body = Mat3::new(
            0.0, 1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0,
        );
        assert_eq!(
            calibration.calibrate_accel(&Vec3::new(2.0, 3.0, 4.0)),
            Vec3::new(3.0, 2.0, 4.0)
        );
    }
}
