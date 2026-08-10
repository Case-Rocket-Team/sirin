use nalgebra::{SMatrix, SVector};

use crate::{
    math::{exp_quaternion, skew},
    state::{
        CovarianceMatrixP, Mat15, NominalState, Vec15, ACCEL_BIAS_INDEX,
        ATTITUDE_INDEX, GYRO_BIAS_INDEX, POSITION_INDEX, VELOCITY_INDEX,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateDecision {
    Accepted,
    InnovationRejected,
    NonFiniteInput,
    InnovationNotPositiveDefinite,
    NonFiniteResult,
    EstimatorTimeUnavailable,
    MeasurementTooOld,
    MeasurementFromFuture,
}

#[derive(Debug, Clone, Copy)]
pub struct UpdateResult {
    pub accepted: bool,
    pub nis: f32,
    pub gate: f32,
    pub correction_norm: f32,
    pub reason: UpdateDecision,
}

impl UpdateResult {
    pub(crate) fn rejected(reason: UpdateDecision, nis: f32, gate: f32) -> Self {
        Self {
            accepted: false,
            nis,
            gate,
            correction_norm: 0.0,
            reason,
        }
    }
}

pub fn update<const M: usize>(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    residual: SVector<f32, M>,
    h: SMatrix<f32, M, 15>,
    measurement_covariance: SMatrix<f32, M, M>,
    gate: f32,
) -> UpdateResult {
    if !residual.iter().all(|value| value.is_finite())
        || !h.iter().all(|value| value.is_finite())
        || !measurement_covariance
            .iter()
            .all(|value| value.is_finite())
    {
        return UpdateResult::rejected(UpdateDecision::NonFiniteInput, f32::NAN, gate);
    }

    let innovation_covariance =
        h * covariance.data * h.transpose() + measurement_covariance;
    let Some(cholesky) = innovation_covariance.cholesky() else {
        return UpdateResult::rejected(
            UpdateDecision::InnovationNotPositiveDefinite,
            f32::NAN,
            gate,
        );
    };

    let weighted_residual = cholesky.solve(&residual);
    let nis = residual.dot(&weighted_residual);
    if !nis.is_finite() {
        return UpdateResult::rejected(UpdateDecision::NonFiniteInput, nis, gate);
    }
    if nis > gate {
        return UpdateResult::rejected(UpdateDecision::InnovationRejected, nis, gate);
    }

    let pht = covariance.data * h.transpose();
    let gain = cholesky.solve(&pht.transpose()).transpose();
    let correction: Vec15 = gain * residual;
    let previous_state = state.clone();
    let previous_covariance = covariance.clone();

    let identity = Mat15::identity();
    let ikh = identity - gain * h;
    covariance.data = ikh * covariance.data * ikh.transpose()
        + gain * measurement_covariance * gain.transpose();
    covariance.symmetrize();

    inject_and_reset(state, covariance, &correction);

    if !covariance.is_finite()
        || !state.pos.iter().all(|value| value.is_finite())
        || !state.vel.iter().all(|value| value.is_finite())
    {
        *state = previous_state;
        *covariance = previous_covariance;
        return UpdateResult::rejected(UpdateDecision::NonFiniteResult, nis, gate);
    }

    UpdateResult {
        accepted: true,
        nis,
        gate,
        correction_norm: correction.norm(),
        reason: UpdateDecision::Accepted,
    }
}

fn inject_and_reset(
    state: &mut NominalState,
    covariance: &mut CovarianceMatrixP,
    correction: &Vec15,
) {
    state.pos += correction.fixed_rows::<3>(POSITION_INDEX).into_owned();
    state.vel += correction.fixed_rows::<3>(VELOCITY_INDEX).into_owned();
    let attitude_correction = correction.fixed_rows::<3>(ATTITUDE_INDEX).into_owned();
    state.rot_quaternion *= exp_quaternion(&attitude_correction);
    state.accel_bias += correction.fixed_rows::<3>(ACCEL_BIAS_INDEX).into_owned();
    state.angular_vel_bias += correction.fixed_rows::<3>(GYRO_BIAS_INDEX).into_owned();

    let mut reset = Mat15::identity();
    reset
        .fixed_view_mut::<3, 3>(ATTITUDE_INDEX, ATTITUDE_INDEX)
        .copy_from(&(crate::state::Mat3::identity() - skew(&attitude_correction) * 0.5));
    covariance.data = reset * covariance.data * reset.transpose();
    covariance.symmetrize();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_update_matches_analytical_result() {
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP {
            data: Mat15::identity(),
        };
        let mut h = SMatrix::<f32, 1, 15>::zeros();
        h[(0, POSITION_INDEX)] = 1.0;

        let result = update(
            &mut state,
            &mut covariance,
            SVector::<f32, 1>::new(2.0),
            h,
            SMatrix::<f32, 1, 1>::new(1.0),
            10.0,
        );

        assert!(result.accepted);
        assert!((state.pos.x - 1.0).abs() < 1.0e-5);
        assert!((covariance.data[(0, 0)] - 0.5).abs() < 1.0e-5);
    }

    #[test]
    fn rejected_update_does_not_change_state_or_covariance() {
        let mut state = NominalState::default();
        let mut covariance = CovarianceMatrixP {
            data: Mat15::identity(),
        };
        let before = covariance.data;
        let mut h = SMatrix::<f32, 1, 15>::zeros();
        h[(0, POSITION_INDEX)] = 1.0;

        let result = update(
            &mut state,
            &mut covariance,
            SVector::<f32, 1>::new(100.0),
            h,
            SMatrix::<f32, 1, 1>::new(1.0),
            9.0,
        );

        assert!(!result.accepted);
        assert_eq!(result.reason, UpdateDecision::InnovationRejected);
        assert_eq!(state.pos, crate::state::Vec3::zeros());
        assert_eq!(covariance.data, before);
    }
}
