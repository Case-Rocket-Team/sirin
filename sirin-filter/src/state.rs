use nalgebra::{SMatrix, SVector, UnitQuaternion};
use uunit::WithUnits;

pub type Scalar = f32;
pub type Vec3 = SVector<Scalar, 3>;
pub type Mat3 = SMatrix<Scalar, 3, 3>;
pub type Vec15 = SVector<Scalar, 15>;
pub type Mat15 = SMatrix<Scalar, 15, 15>;

pub const ERROR_DIM: usize = 15;
pub const POSITION_INDEX: usize = 0;
pub const VELOCITY_INDEX: usize = 3;
pub const ATTITUDE_INDEX: usize = 6;
pub const ACCEL_BIAS_INDEX: usize = 9;
pub const GYRO_BIAS_INDEX: usize = 12;

/// Nominal navigation state. Position, velocity, and acceleration are NED.
#[derive(Debug, Clone)]
pub struct NominalState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub rot_quaternion: UnitQuaternion<Scalar>,
    pub accel_bias: Vec3,
    pub angular_vel_bias: Vec3,
    pub accel: Vec3,
}

impl Default for NominalState {
    fn default() -> Self {
        Self {
            pos: Vec3::zeros(),
            vel: Vec3::zeros(),
            rot_quaternion: UnitQuaternion::identity(),
            accel_bias: Vec3::zeros(),
            angular_vel_bias: Vec3::zeros(),
            accel: Vec3::zeros(),
        }
    }
}

impl From<&NominalState> for sirin_shared::state::NominalState {
    fn from(state: &NominalState) -> Self {
        Self {
            pos: sirin_shared::state::Pos {
                x: state.pos.x.with_units(),
                y: state.pos.y.with_units(),
                z: state.pos.z.with_units(),
            },
            vel: sirin_shared::state::Vel {
                x: state.vel.x.with_units(),
                y: state.vel.y.with_units(),
                z: state.vel.z.with_units(),
            },
            accel: sirin_shared::state::Accel {
                x: state.accel.x.with_units(),
                y: state.accel.y.with_units(),
                z: state.accel.z.with_units(),
            },
            rot_quaternion: sirin_shared::state::Quaternion::new(
                state.rot_quaternion.w,
                state.rot_quaternion.i,
                state.rot_quaternion.j,
                state.rot_quaternion.k,
            ),
            accel_bias: sirin_shared::state::Accel {
                x: state.accel_bias.x.with_units(),
                y: state.accel_bias.y.with_units(),
                z: state.accel_bias.z.with_units(),
            },
            angular_vel_bias: sirin_shared::state::AngularVel {
                x_pitch: state.angular_vel_bias.x.with_units(),
                y_roll: state.angular_vel_bias.y.with_units(),
                z_yaw: state.angular_vel_bias.z.with_units(),
            },
        }
    }
}

/// Compatibility representation of an error-state mean.
///
/// The ESKF normally keeps this mean at zero and injects each correction
/// immediately. It remains public for existing telemetry and example callers.
#[derive(Debug, Clone, Default)]
pub struct ErrorState {
    pub pos: Vec3,
    pub vel: Vec3,
    pub angles_vector: Vec3,
    pub accel_bias: Vec3,
    pub angular_vel_bias: Vec3,
}

impl ErrorState {
    pub fn as_vector(&self) -> Vec15 {
        let mut value = Vec15::zeros();
        value.fixed_rows_mut::<3>(POSITION_INDEX).copy_from(&self.pos);
        value.fixed_rows_mut::<3>(VELOCITY_INDEX).copy_from(&self.vel);
        value
            .fixed_rows_mut::<3>(ATTITUDE_INDEX)
            .copy_from(&self.angles_vector);
        value
            .fixed_rows_mut::<3>(ACCEL_BIAS_INDEX)
            .copy_from(&self.accel_bias);
        value
            .fixed_rows_mut::<3>(GYRO_BIAS_INDEX)
            .copy_from(&self.angular_vel_bias);
        value
    }
}

#[derive(Debug, Clone)]
pub struct CovarianceMatrixP {
    pub data: Mat15,
}

impl CovarianceMatrixP {
    pub fn from_initial_uncertainty(
        uncertainty: &crate::config::InitialUncertainty,
    ) -> Self {
        let mut data = Mat15::zeros();
        set_diagonal3(&mut data, POSITION_INDEX, &uncertainty.position_std_m);
        set_diagonal3(&mut data, VELOCITY_INDEX, &uncertainty.velocity_std_mps);
        set_diagonal3(
            &mut data,
            ATTITUDE_INDEX,
            &uncertainty.attitude_std_rad,
        );
        set_diagonal3(
            &mut data,
            ACCEL_BIAS_INDEX,
            &uncertainty.accel_bias_std_mps2,
        );
        set_diagonal3(
            &mut data,
            GYRO_BIAS_INDEX,
            &uncertainty.gyro_bias_std_radps,
        );
        Self { data }
    }

    pub fn is_finite(&self) -> bool {
        self.data.iter().all(|value| value.is_finite())
    }

    pub fn symmetrize(&mut self) {
        self.data = (self.data + self.data.transpose()) * 0.5;
    }
}

impl Default for CovarianceMatrixP {
    fn default() -> Self {
        Self::from_initial_uncertainty(&crate::config::InitialUncertainty::default())
    }
}

fn set_diagonal3(matrix: &mut Mat15, start: usize, standard_deviation: &Vec3) {
    for axis in 0..3 {
        matrix[(start + axis, start + axis)] =
            standard_deviation[axis] * standard_deviation[axis];
    }
}
