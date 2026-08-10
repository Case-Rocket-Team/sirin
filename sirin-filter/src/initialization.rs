use nalgebra::UnitQuaternion;

use crate::{
    config::EskfConfig,
    state::{NominalState, Vec3},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InitializationError {
    NotEnoughSamples,
    NonFiniteSample,
    VehicleMoving,
    InvalidGravityVector,
}

/// Fixed-memory prelaunch stationary initializer.
#[derive(Debug, Clone)]
pub struct StationaryInitializer {
    required_samples: u32,
    count: u32,
    accel_sum: Vec3,
    gyro_sum: Vec3,
    accel_norm_sum: f32,
    accel_norm_squared_sum: f32,
    gyro_norm_squared_sum: f32,
    max_accel_std_mps2: f32,
    max_gyro_rms_radps: f32,
    invalid_sample_seen: bool,
}

impl StationaryInitializer {
    pub fn new(required_samples: u32) -> Self {
        Self {
            required_samples,
            count: 0,
            accel_sum: Vec3::zeros(),
            gyro_sum: Vec3::zeros(),
            accel_norm_sum: 0.0,
            accel_norm_squared_sum: 0.0,
            gyro_norm_squared_sum: 0.0,
            max_accel_std_mps2: 0.15,
            max_gyro_rms_radps: 0.03,
            invalid_sample_seen: false,
        }
    }

    pub fn push(&mut self, accel_mps2_b: &Vec3, gyro_radps_b: &Vec3) {
        if !accel_mps2_b.iter().all(|value| value.is_finite())
            || !gyro_radps_b.iter().all(|value| value.is_finite())
        {
            self.invalid_sample_seen = true;
            return;
        }
        let accel_norm = accel_mps2_b.norm();
        self.count = self.count.saturating_add(1);
        self.accel_sum += accel_mps2_b;
        self.gyro_sum += gyro_radps_b;
        self.accel_norm_sum += accel_norm;
        self.accel_norm_squared_sum += accel_norm * accel_norm;
        self.gyro_norm_squared_sum += gyro_radps_b.norm_squared();
    }

    pub fn sample_count(&self) -> u32 {
        self.count
    }

    pub fn try_initialize(
        &self,
        config: &EskfConfig,
    ) -> Result<NominalState, InitializationError> {
        if self.invalid_sample_seen {
            return Err(InitializationError::NonFiniteSample);
        }
        if self.count < self.required_samples || self.count == 0 {
            return Err(InitializationError::NotEnoughSamples);
        }

        let count = self.count as f32;
        let mean_accel = self.accel_sum / count;
        let mean_gyro = self.gyro_sum / count;
        let mean_accel_norm = self.accel_norm_sum / count;
        let accel_variance =
            (self.accel_norm_squared_sum / count - mean_accel_norm * mean_accel_norm)
                .max(0.0);
        let gyro_rms = libm::sqrtf(self.gyro_norm_squared_sum / count);

        if libm::sqrtf(accel_variance) > self.max_accel_std_mps2
            || gyro_rms > self.max_gyro_rms_radps
            || (mean_accel_norm - config.gravity_ned_mps2.norm()).abs() > 0.75
        {
            return Err(InitializationError::VehicleMoving);
        }

        let Some(attitude_nb) = UnitQuaternion::rotation_between(
            &mean_accel,
            &Vec3::new(0.0, 0.0, -config.gravity_ned_mps2.norm()),
        ) else {
            return Err(InitializationError::InvalidGravityVector);
        };

        Ok(NominalState {
            rot_quaternion: attitude_nb,
            angular_vel_bias: mean_gyro,
            ..NominalState::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn initializes_level_stationary_state() {
        let config = EskfConfig::default();
        let mut initializer = StationaryInitializer::new(100);
        for _ in 0..100 {
            initializer.push(
                &Vec3::new(0.0, 0.0, -9.80665),
                &Vec3::new(0.001, -0.002, 0.003),
            );
        }

        let state = initializer.try_initialize(&config).unwrap();
        assert!(state.rot_quaternion.angle() < 1.0e-5);
        assert!((state.angular_vel_bias - Vec3::new(0.001, -0.002, 0.003)).norm() < 1.0e-6);
    }

    #[test]
    fn rejects_motion() {
        let config = EskfConfig::default();
        let mut initializer = StationaryInitializer::new(2);
        initializer.push(&Vec3::new(0.0, 0.0, -9.80665), &Vec3::zeros());
        initializer.push(&Vec3::new(0.0, 0.0, -5.0), &Vec3::new(1.0, 0.0, 0.0));
        assert!(matches!(
            initializer.try_initialize(&config),
            Err(InitializationError::VehicleMoving)
        ));
    }
}
