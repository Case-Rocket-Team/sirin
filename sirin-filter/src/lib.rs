#![no_std]

pub mod config;
pub mod calibration;
pub mod coordinates;
pub mod health;
pub mod initialization;
pub mod history;
pub mod math;
pub mod measurement;
pub mod propagation;
pub mod runtime;
pub mod state;
pub mod update;

pub use config::{EskfConfig, ImuNoise, InitialUncertainty};
pub use calibration::SensorCalibration;
pub use health::{AidingHealth, AidingStatus, EstimatorHealth};
pub use initialization::{InitializationError, StationaryInitializer};
pub use history::{FixedLagHistory, HistoryProcessResult};
pub use measurement::{
    BarometerObservation, GpsPositionObservation, GpsVelocityObservation,
    ImuSample, MagnetometerObservation,
};
pub use propagation::{propagate, PropagationError};
pub use runtime::{
    EstimatorDiagnostics, EstimatorEvent, EstimatorRuntime, ProcessResult,
};
pub use state::{
    CovarianceMatrixP, ErrorState, Mat15, NominalState, Vec15, Vec3, ERROR_DIM,
};
pub use update::{UpdateDecision, UpdateResult};

use state::ErrorState as LegacyErrorState;

/// Complete 15-state error-state Kalman filter.
#[derive(Debug, Clone)]
pub struct Eskf {
    pub state: NominalState,
    pub covariance: CovarianceMatrixP,
    pub config: EskfConfig,
    last_imu: Option<measurement::ImuSample>,
}

impl Eskf {
    pub fn new(state: NominalState, covariance: CovarianceMatrixP, config: EskfConfig) -> Self {
        Self {
            state,
            covariance,
            config,
            last_imu: None,
        }
    }

    pub fn from_initialized_state(state: NominalState, config: EskfConfig) -> Self {
        let covariance = CovarianceMatrixP::from_initial_uncertainty(&config.initial_uncertainty);
        Self::new(state, covariance, config)
    }

    /// Propagate to an IMU sample's physical capture timestamp.
    ///
    /// The first sample establishes the integration epoch and does not move the
    /// state. Subsequent calls use midpoint IMU values.
    pub fn propagate_imu(
        &mut self,
        sample: measurement::ImuSample,
    ) -> Result<(), PropagationError> {
        if sample.accel_saturated.iter().any(|value| *value)
            || sample.gyro_saturated.iter().any(|value| *value)
        {
            return Err(PropagationError::SaturatedInput);
        }
        let Some(previous) = self.last_imu else {
            self.last_imu = Some(sample);
            return Ok(());
        };

        if sample.timestamp_us <= previous.timestamp_us {
            return Err(PropagationError::NonMonotonicTimestamp);
        }
        if sample.sequence != previous.sequence.wrapping_add(1) {
            return Err(PropagationError::SequenceDiscontinuity);
        }

        let dt = (sample.timestamp_us - previous.timestamp_us) as f32 * 1.0e-6;
        let accel = (previous.accel_mps2_b + sample.accel_mps2_b) * 0.5;
        let gyro = (previous.gyro_radps_b + sample.gyro_radps_b) * 0.5;

        let result = propagate(
            &mut self.state,
            &mut self.covariance,
            dt,
            &accel,
            &gyro,
            &self.config,
        );

        if result.is_ok() {
            self.last_imu = Some(sample);
        }
        result
    }

    pub fn fuse_gps_velocity(
        &mut self,
        observation: &GpsVelocityObservation,
    ) -> UpdateResult {
        if let Some(rejected) = self.check_measurement_time(observation.timestamp_us) {
            return rejected;
        }
        measurement::fuse_gps_velocity(
            &mut self.state,
            &mut self.covariance,
            observation,
            self.config.gps_velocity_gate,
        )
    }

    pub fn fuse_gps_position(
        &mut self,
        observation: &GpsPositionObservation,
    ) -> UpdateResult {
        if let Some(rejected) = self.check_measurement_time(observation.timestamp_us) {
            return rejected;
        }
        measurement::fuse_gps_position(
            &mut self.state,
            &mut self.covariance,
            observation,
            self.config.gps_position_gate,
        )
    }

    pub fn fuse_barometer(
        &mut self,
        observation: &BarometerObservation,
    ) -> UpdateResult {
        if let Some(rejected) = self.check_measurement_time(observation.timestamp_us) {
            return rejected;
        }
        measurement::fuse_barometer(
            &mut self.state,
            &mut self.covariance,
            observation,
            self.config.barometer_gate,
        )
    }

    pub fn fuse_magnetometer(
        &mut self,
        observation: &MagnetometerObservation,
    ) -> UpdateResult {
        if let Some(rejected) = self.check_measurement_time(observation.timestamp_us) {
            return rejected;
        }
        measurement::fuse_magnetometer(
            &mut self.state,
            &mut self.covariance,
            observation,
            &self.config.magnetic_field_ned_ut,
            self.config.magnetometer_gate,
        )
    }

    pub fn timestamp_us(&self) -> Option<u64> {
        self.last_imu.map(|sample| sample.timestamp_us)
    }

    pub fn navigation_solution(
        &self,
    ) -> sirin_shared::estimator::NavigationSolution {
        let quaternion = self.state.rot_quaternion.clone();
        sirin_shared::estimator::NavigationSolution {
            timestamp_us: self.timestamp_us().unwrap_or(0),
            position_ned_m: [
                self.state.pos.x,
                self.state.pos.y,
                self.state.pos.z,
            ],
            velocity_ned_mps: [
                self.state.vel.x,
                self.state.vel.y,
                self.state.vel.z,
            ],
            attitude_nb_wxyz: [quaternion.w, quaternion.i, quaternion.j, quaternion.k],
            accel_bias_mps2_b: [
                self.state.accel_bias.x,
                self.state.accel_bias.y,
                self.state.accel_bias.z,
            ],
            gyro_bias_radps_b: [
                self.state.angular_vel_bias.x,
                self.state.angular_vel_bias.y,
                self.state.angular_vel_bias.z,
            ],
            validity: sirin_shared::estimator::NavigationValidity {
                attitude: self.timestamp_us().is_some(),
                covariance: self.covariance.is_finite(),
                time_sync: self.timestamp_us().is_some(),
                ..Default::default()
            },
        }
    }

    fn check_measurement_time(&self, timestamp_us: u64) -> Option<UpdateResult> {
        let Some(current_time_us) = self.timestamp_us() else {
            return Some(UpdateResult::rejected(
                UpdateDecision::EstimatorTimeUnavailable,
                f32::NAN,
                0.0,
            ));
        };
        if timestamp_us
            > current_time_us.saturating_add(self.config.future_measurement_tolerance_us)
        {
            return Some(UpdateResult::rejected(
                UpdateDecision::MeasurementFromFuture,
                f32::NAN,
                0.0,
            ));
        }
        if current_time_us.saturating_sub(timestamp_us) > self.config.max_measurement_age_us {
            return Some(UpdateResult::rejected(
                UpdateDecision::MeasurementTooOld,
                f32::NAN,
                0.0,
            ));
        }
        None
    }

    pub(crate) fn fuse_gps_position_unchecked(
        &mut self,
        observation: &GpsPositionObservation,
    ) -> UpdateResult {
        measurement::fuse_gps_position(
            &mut self.state,
            &mut self.covariance,
            observation,
            self.config.gps_position_gate,
        )
    }

    pub(crate) fn fuse_gps_velocity_unchecked(
        &mut self,
        observation: &GpsVelocityObservation,
    ) -> UpdateResult {
        measurement::fuse_gps_velocity(
            &mut self.state,
            &mut self.covariance,
            observation,
            self.config.gps_velocity_gate,
        )
    }
}

impl Default for Eskf {
    fn default() -> Self {
        Self::new(
            NominalState::default(),
            CovarianceMatrixP::default(),
            EskfConfig::default(),
        )
    }
}

/// Compatibility wrapper for the original prototype API.
pub fn update_with_imu(
    nominal: &mut NominalState,
    _error: &mut LegacyErrorState,
    covariance: &mut CovarianceMatrixP,
    dt: f32,
    accel_measurement: &Vec3,
    gyro_measurement: &Vec3,
) {
    let _ = propagate(
        nominal,
        covariance,
        dt,
        accel_measurement,
        gyro_measurement,
        &EskfConfig::default(),
    );
}

/// Compatibility wrapper for callers that already provide local NED GPS data.
pub fn correct_from_gps(
    nominal: &mut NominalState,
    _error: &mut LegacyErrorState,
    covariance: &mut CovarianceMatrixP,
    gps_position_ned_m: &Vec3,
    gps_velocity_ned_mps: &Vec3,
) {
    let config = EskfConfig::default();
    let velocity = GpsVelocityObservation {
        timestamp_us: 0,
        velocity_ned_mps: *gps_velocity_ned_mps,
        variance_m2ps2: config.gps_velocity_variance_m2ps2,
    };
    let position = GpsPositionObservation {
        timestamp_us: 0,
        position_ned_m: *gps_position_ned_m,
        variance_m2: config.gps_position_variance_m2,
    };
    let _ = measurement::fuse_gps_velocity(
        nominal,
        covariance,
        &velocity,
        config.gps_velocity_gate,
    );
    let _ = measurement::fuse_gps_position(
        nominal,
        covariance,
        &position,
        config.gps_position_gate,
    );
}

/// The generic update injects and resets immediately, so this is now a no-op.
pub fn converge_states(
    _nominal: &mut NominalState,
    error: &mut LegacyErrorState,
    _covariance: &mut CovarianceMatrixP,
) {
    *error = LegacyErrorState::default();
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterError {
    InitNoAccel,
}

/// Compatibility initialization entry point.
///
/// New code should use StationaryInitializer, which rejects motion using a
/// window of measurements. This function performs gravity alignment only.
pub fn init_with_imu(
    nominal: &mut NominalState,
    accel_measurement_mps2_b: &Vec3,
    _gyro_measurement_radps_b: &Vec3,
    _magnetometer_measurement_ut_b: &Vec3,
) -> Result<(), FilterError> {
    let gravity = EskfConfig::default().gravity_ned_mps2.norm();
    if accel_measurement_mps2_b.norm() < 1.0e-3 {
        return Err(FilterError::InitNoAccel);
    }
    nominal.rot_quaternion = nalgebra::UnitQuaternion::rotation_between(
        accel_measurement_mps2_b,
        &Vec3::new(0.0, 0.0, -gravity),
    )
    .ok_or(FilterError::InitNoAccel)?;
    nominal.pos = Vec3::zeros();
    nominal.vel = Vec3::zeros();
    Ok(())
}

/// Compatibility wrapper using the configured launch-site magnetic field.
pub fn fuse_magnetometer(
    nominal: &mut NominalState,
    _error: &mut LegacyErrorState,
    covariance: &mut CovarianceMatrixP,
    field_ut_b: &Vec3,
) {
    let config = EskfConfig::default();
    let observation = MagnetometerObservation {
        timestamp_us: 0,
        field_ut_b: *field_ut_b,
        variance_ut2: Vec3::repeat(4.0),
    };
    let _ = measurement::fuse_magnetometer(
        nominal,
        covariance,
        &observation,
        &config.magnetic_field_ned_ut,
        config.magnetometer_gate,
    );
}
