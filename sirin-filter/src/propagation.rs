use nalgebra::SMatrix;

use crate::{
    config::EskfConfig,
    math::{all_finite3, exp_quaternion, skew},
    state::{
        CovarianceMatrixP, Mat15, NominalState, Vec3, ACCEL_BIAS_INDEX,
        ATTITUDE_INDEX, GYRO_BIAS_INDEX, POSITION_INDEX, VELOCITY_INDEX,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropagationError {
    InvalidTimeStep,
    NonMonotonicTimestamp,
    SequenceDiscontinuity,
    NonFiniteInput,
    NonFiniteState,
    InvalidCovariance,
    SaturatedInput,
}

pub fn propagate(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    dt_s: f32,
    accel_measurement_mps2_b: &Vec3,
    gyro_measurement_radps_b: &Vec3,
    config: &EskfConfig,
) -> Result<(), PropagationError> {
    if !dt_s.is_finite()
        || dt_s < config.min_imu_dt_s
        || dt_s > config.max_imu_dt_s
    {
        return Err(PropagationError::InvalidTimeStep);
    }
    if !all_finite3(accel_measurement_mps2_b)
        || !all_finite3(gyro_measurement_radps_b)
    {
        return Err(PropagationError::NonFiniteInput);
    }

    let previous_state = state.clone();
    let previous_covariance = covariance.clone();
    let specific_force_b = accel_measurement_mps2_b - state.accel_bias;
    let angular_rate_b = gyro_measurement_radps_b - state.angular_vel_bias;
    let half_delta = exp_quaternion(&(angular_rate_b * (0.5 * dt_s)));
    let midpoint_attitude = state.rot_quaternion * half_delta;
    let midpoint_rotation = *midpoint_attitude.to_rotation_matrix().matrix();
    let acceleration_ned =
        midpoint_attitude.transform_vector(&specific_force_b) + config.gravity_ned_mps2;

    state.pos += state.vel * dt_s + acceleration_ned * (0.5 * dt_s * dt_s);
    state.vel += acceleration_ned * dt_s;
    state.rot_quaternion *= exp_quaternion(&(angular_rate_b * dt_s));
    state.accel = acceleration_ned;

    if !state_is_finite(state) {
        *state = previous_state.clone();
        return Err(PropagationError::NonFiniteState);
    }

    propagate_covariance(
        covariance,
        dt_s,
        &specific_force_b,
        &angular_rate_b,
        &midpoint_rotation,
        config,
    );

    if !covariance.is_finite()
        || (0..15).any(|index| covariance.data[(index, index)] < -1.0e-6)
    {
        *state = previous_state;
        *covariance = previous_covariance;
        return Err(PropagationError::InvalidCovariance);
    }
    Ok(())
}

fn propagate_covariance(
    covariance: &mut CovarianceMatrixP,
    dt_s: f32,
    specific_force_b: &Vec3,
    angular_rate_b: &Vec3,
    rotation_nb: &crate::state::Mat3,
    config: &EskfConfig,
) {
    let mut f = Mat15::zeros();
    f.fixed_view_mut::<3, 3>(POSITION_INDEX, VELOCITY_INDEX)
        .copy_from(&crate::state::Mat3::identity());
    f.fixed_view_mut::<3, 3>(VELOCITY_INDEX, ATTITUDE_INDEX)
        .copy_from(&(-rotation_nb * skew(specific_force_b)));
    f.fixed_view_mut::<3, 3>(VELOCITY_INDEX, ACCEL_BIAS_INDEX)
        .copy_from(&(-rotation_nb));
    f.fixed_view_mut::<3, 3>(ATTITUDE_INDEX, ATTITUDE_INDEX)
        .copy_from(&(-skew(angular_rate_b)));
    f.fixed_view_mut::<3, 3>(ATTITUDE_INDEX, GYRO_BIAS_INDEX)
        .copy_from(&(-crate::state::Mat3::identity()));

    let f_dt = f * dt_s;
    let transition = Mat15::identity() + f_dt + (f_dt * f_dt) * 0.5;

    // Noise order: accelerometer, gyro, accel-bias RW, gyro-bias RW.
    let mut g = SMatrix::<f32, 15, 12>::zeros();
    g.fixed_view_mut::<3, 3>(VELOCITY_INDEX, 0)
        .copy_from(&(-rotation_nb));
    g.fixed_view_mut::<3, 3>(ATTITUDE_INDEX, 3)
        .copy_from(&(-crate::state::Mat3::identity()));
    g.fixed_view_mut::<3, 3>(ACCEL_BIAS_INDEX, 6)
        .copy_from(&crate::state::Mat3::identity());
    g.fixed_view_mut::<3, 3>(GYRO_BIAS_INDEX, 9)
        .copy_from(&crate::state::Mat3::identity());

    let mut qc = SMatrix::<f32, 12, 12>::zeros();
    set_noise_variance(
        &mut qc,
        0,
        &config.imu_noise.accel_noise_density_mps2_sqrt_hz,
    );
    set_noise_variance(
        &mut qc,
        3,
        &config.imu_noise.gyro_noise_density_radps_sqrt_hz,
    );
    set_noise_variance(
        &mut qc,
        6,
        &config.imu_noise.accel_bias_random_walk_mps3_sqrt_hz,
    );
    set_noise_variance(
        &mut qc,
        9,
        &config.imu_noise.gyro_bias_random_walk_radps2_sqrt_hz,
    );

    let discrete_noise = g * qc * g.transpose() * dt_s;
    covariance.data =
        transition * covariance.data * transition.transpose() + discrete_noise;
    covariance.symmetrize();
}

fn set_noise_variance(
    matrix: &mut SMatrix<f32, 12, 12>,
    start: usize,
    density: &Vec3,
) {
    for axis in 0..3 {
        matrix[(start + axis, start + axis)] = density[axis] * density[axis];
    }
}

fn state_is_finite(state: &NominalState) -> bool {
    all_finite3(&state.pos)
        && all_finite3(&state.vel)
        && all_finite3(&state.accel_bias)
        && all_finite3(&state.angular_vel_bias)
        && state.rot_quaternion.coords.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn static_level_state_remains_stationary() {
        let config = EskfConfig::default();
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP::default();
        let specific_force = Vec3::new(0.0, 0.0, -9.80665);

        for _ in 0..1000 {
            propagate(
                &mut state,
                &mut covariance,
                0.001,
                &specific_force,
                &Vec3::zeros(),
                &config,
            )
            .unwrap();
        }

        assert!(state.pos.norm() < 1.0e-4);
        assert!(state.vel.norm() < 1.0e-4);
        assert!((state.rot_quaternion.angle()).abs() < 1.0e-5);
        assert!(covariance.is_finite());
    }

    #[test]
    fn integrates_constant_yaw_rate() {
        let config = EskfConfig::default();
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP::default();

        for _ in 0..100 {
            propagate(
                &mut state,
                &mut covariance,
                0.01,
                &Vec3::new(0.0, 0.0, -9.80665),
                &Vec3::new(0.0, 0.0, 1.0),
                &config,
            )
            .unwrap();
        }

        assert!((state.rot_quaternion.angle() - 1.0).abs() < 2.0e-4);
    }
}
