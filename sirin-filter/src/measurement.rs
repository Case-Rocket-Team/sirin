use nalgebra::{SMatrix, SVector};

use crate::{
    math::skew,
    state::{
        CovarianceMatrixP, NominalState, Vec3, ATTITUDE_INDEX, POSITION_INDEX,
        VELOCITY_INDEX,
    },
    update::{update, UpdateResult},
};

#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub timestamp_us: u64,
    pub accel_mps2_b: Vec3,
    pub gyro_radps_b: Vec3,
    pub temperature_c: Option<f32>,
    pub accel_saturated: [bool; 3],
    pub gyro_saturated: [bool; 3],
    pub sequence: u32,
}

impl From<&sirin_shared::estimator::ImuSample> for ImuSample {
    fn from(sample: &sirin_shared::estimator::ImuSample) -> Self {
        Self {
            timestamp_us: sample.time.timestamp_us,
            accel_mps2_b: Vec3::from_row_slice(&sample.accel_mps2_b),
            gyro_radps_b: Vec3::from_row_slice(&sample.gyro_radps_b),
            temperature_c: sample.temperature_c,
            accel_saturated: [
                sample.accel_saturation_mask & 0b001 != 0,
                sample.accel_saturation_mask & 0b010 != 0,
                sample.accel_saturation_mask & 0b100 != 0,
            ],
            gyro_saturated: [
                sample.gyro_saturation_mask & 0b001 != 0,
                sample.gyro_saturation_mask & 0b010 != 0,
                sample.gyro_saturation_mask & 0b100 != 0,
            ],
            sequence: sample.time.sequence,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GpsPositionObservation {
    pub timestamp_us: u64,
    pub position_ned_m: Vec3,
    pub variance_m2: Vec3,
}

#[derive(Debug, Clone, Copy)]
pub struct GpsVelocityObservation {
    pub timestamp_us: u64,
    pub velocity_ned_mps: Vec3,
    pub variance_m2ps2: Vec3,
}

impl GpsVelocityObservation {
    /// Convert the compact receiver packet into SI NED velocity.
    ///
    /// Receiver velocity and accuracy are stored in centimeters per second to
    /// keep telemetry packets small.
    pub fn from_gps_fix(
        timestamp_us: u64,
        fix: &sirin_shared::packet::GpsFix,
        variance_floor_m2ps2: &Vec3,
    ) -> Option<Self> {
        use sirin_shared::packet::GpsFixType;

        if !matches!(
            fix.fix_type,
            GpsFixType::Fix3d | GpsFixType::FixDifferential
        ) {
            return None;
        }
        let velocity_ned_mps = Vec3::new(
            fix.vel.x.value as f32 * 0.01,
            fix.vel.y.value as f32 * 0.01,
            fix.vel.z.value as f32 * 0.01,
        );
        let speed_std_mps = fix.vel_acc.value as f32 * 0.01;
        let receiver_variance = speed_std_mps * speed_std_mps;
        let variance_m2ps2 = Vec3::new(
            receiver_variance.max(variance_floor_m2ps2.x),
            receiver_variance.max(variance_floor_m2ps2.y),
            receiver_variance.max(variance_floor_m2ps2.z),
        );
        Some(Self {
            timestamp_us,
            velocity_ned_mps,
            variance_m2ps2,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BarometerObservation {
    pub timestamp_us: u64,
    /// Local height above the initialized reference, positive upward.
    pub height_up_m: f32,
    pub variance_m2: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct MagnetometerObservation {
    pub timestamp_us: u64,
    pub field_ut_b: Vec3,
    pub variance_ut2: Vec3,
}

pub fn fuse_gps_position(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    observation: &GpsPositionObservation,
    gate: f32,
) -> UpdateResult {
    let residual = observation.position_ned_m - state.pos;
    let mut h = SMatrix::<f32, 3, 15>::zeros();
    h.fixed_view_mut::<3, 3>(0, POSITION_INDEX)
        .copy_from(&crate::state::Mat3::identity());
    update(
        state,
        covariance,
        residual,
        h,
        diagonal3(&observation.variance_m2),
        gate,
    )
}

pub fn fuse_gps_velocity(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    observation: &GpsVelocityObservation,
    gate: f32,
) -> UpdateResult {
    let residual = observation.velocity_ned_mps - state.vel;
    let mut h = SMatrix::<f32, 3, 15>::zeros();
    h.fixed_view_mut::<3, 3>(0, VELOCITY_INDEX)
        .copy_from(&crate::state::Mat3::identity());
    update(
        state,
        covariance,
        residual,
        h,
        diagonal3(&observation.variance_m2ps2),
        gate,
    )
}

pub fn fuse_barometer(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    observation: &BarometerObservation,
    gate: f32,
) -> UpdateResult {
    let predicted_height_up_m = -state.pos.z;
    let residual = SVector::<f32, 1>::new(
        observation.height_up_m - predicted_height_up_m,
    );
    let mut h = SMatrix::<f32, 1, 15>::zeros();
    h[(0, POSITION_INDEX + 2)] = -1.0;
    update(
        state,
        covariance,
        residual,
        h,
        SMatrix::<f32, 1, 1>::new(observation.variance_m2),
        gate,
    )
}

pub fn fuse_magnetometer(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    observation: &MagnetometerObservation,
    expected_field_ned_ut: &Vec3,
    gate: f32,
) -> UpdateResult {
    let predicted_field_b = state
        .rot_quaternion
        .inverse_transform_vector(expected_field_ned_ut);
    let residual = observation.field_ut_b - predicted_field_b;
    let mut h = SMatrix::<f32, 3, 15>::zeros();
    // For R_true = R_nominal Exp(delta_theta), R_true^T m_N is
    // approximately m_B + [m_B]x delta_theta.
    h.fixed_view_mut::<3, 3>(0, ATTITUDE_INDEX)
        .copy_from(&skew(&predicted_field_b));
    update(
        state,
        covariance,
        residual,
        h,
        diagonal3(&observation.variance_ut2),
        gate,
    )
}

fn diagonal3(diagonal: &Vec3) -> SMatrix<f32, 3, 3> {
    SMatrix::<f32, 3, 3>::from_diagonal(diagonal)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_position_moves_state_toward_measurement() {
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP::default();
        let observation = GpsPositionObservation {
            timestamp_us: 1,
            position_ned_m: Vec3::new(3.0, -2.0, 1.0),
            variance_m2: Vec3::repeat(1.0),
        };

        let result = fuse_gps_position(
            &mut state,
            &mut covariance,
            &observation,
            14.16,
        );

        assert!(result.accepted);
        assert!(state.pos.x > 0.0 && state.pos.x < 3.0);
        assert!(state.pos.y < 0.0 && state.pos.y > -2.0);
    }

    #[test]
    fn barometer_uses_up_positive_height() {
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP::default();
        let observation = BarometerObservation {
            timestamp_us: 1,
            height_up_m: 5.0,
            variance_m2: 1.0,
        };

        let result = fuse_barometer(&mut state, &mut covariance, &observation, 9.0);

        assert!(result.accepted);
        assert!(state.pos.z < 0.0);
    }

    #[test]
    fn receiver_velocity_converts_centimeters_to_meters() {
        use uunit::WithUnits;

        let mut fix = sirin_shared::packet::GpsFix::default();
        fix.fix_type = sirin_shared::packet::GpsFixType::Fix3d;
        fix.vel.x = 125i32.with_units();
        fix.vel.y = (-50i32).with_units();
        fix.vel.z = 25i32.with_units();
        fix.vel_acc = 10u32.with_units();

        let observation = GpsVelocityObservation::from_gps_fix(
            42,
            &fix,
            &Vec3::repeat(0.02),
        )
        .unwrap();

        assert!((observation.velocity_ned_mps.x - 1.25).abs() < 1.0e-6);
        assert!((observation.velocity_ned_mps.y + 0.5).abs() < 1.0e-6);
        assert!((observation.velocity_ned_mps.z - 0.25).abs() < 1.0e-6);
        assert!((observation.variance_m2ps2.x - 0.02).abs() < 1.0e-6);
    }

    #[test]
    fn magnetometer_full_vector_update_is_applied() {
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP::default();
        let expected_field_ned = Vec3::new(19.0, -2.7, 48.0);
        let true_attitude =
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, 0.2);
        let measured_field_b =
            true_attitude.inverse_transform_vector(&expected_field_ned);
        let observation = MagnetometerObservation {
            timestamp_us: 1,
            field_ut_b: measured_field_b,
            variance_ut2: Vec3::repeat(0.25),
        };

        let result = fuse_magnetometer(
            &mut state,
            &mut covariance,
            &observation,
            &expected_field_ned,
            14.16,
        );

        assert!(result.accepted);
        assert!(result.correction_norm > 0.0);
    }
}
