#![no_std]

use nalgebra as na;
use na::{Matrix3, Matrix6, Vector3, UnitQuaternion, Rotation3};
use libm::{cosf, sinf, sqrtf, fabsf, powf};
use defmt;

// Constants
const DEBUG: bool = true;
const GRAVITY: f32 = 9.80665;
const STATE_DIMS: usize = 18;
const EARTH_RADIUS: f32 = 6378137.0;

const VEL_NOISE: f32 = 1.0;
const ANGLES_NOISE: f32 = 1.0;
const ACCEL_NOISE: f32 = 1.0;
const ANGULAR_VEL_NOISE: f32 = 1.0;


pub enum FilterError {
    InitNoAccel
}

/// Nominal state structure
#[derive(Debug, Clone)]
pub struct NominalState {
    pub pos: Vector3<f32>,
    pub vel: Vector3<f32>,
    pub accel: Vector3<f32>,
    pub rot_quaternion: UnitQuaternion<f32>,
    pub accel_bias: Vector3<f32>,
    pub angular_vel_bias: Vector3<f32>,
}

impl Default for NominalState {
    fn default() -> Self {
        Self {
            pos: Vector3::new(EARTH_RADIUS, 0.0, 0.0),
            vel: Vector3::zeros(),
            accel: Vector3::zeros(),
            rot_quaternion: UnitQuaternion::identity(),
            accel_bias: Vector3::zeros(),
            angular_vel_bias: Vector3::zeros(),
        }
    }
}

/// Error state structure
#[derive(Debug, Clone, Default)]
pub struct ErrorState {
    pub pos: Vector3<f32>,
    pub vel: Vector3<f32>,
    pub accel: Vector3<f32>,
    pub accel_bias: Vector3<f32>,
    pub angular_vel_bias: Vector3<f32>,
    pub angles_vector: Vector3<f32>,
}

/// Covariance matrix P (18x18)
#[derive(Debug, Clone)]
pub struct CovarianceMatrixP {
    pub data: na::SMatrix<f32, STATE_DIMS, STATE_DIMS>,
}

impl Default for CovarianceMatrixP {
    fn default() -> Self {
        Self {
            data: na::SMatrix::<f32, STATE_DIMS, STATE_DIMS>::zeros(),
        }
    }
}

/// Calculate pressure altitude from pressure in hPa
pub fn pressure_altitude(pressure_hpa: f64) -> f64 {
    // TODO: Implement actual calculation
    // 44307.7 - 11872.4 * pressure_hpa.powf(0.190284)
    0.0
}

/// Calculate gravity at a given altitude
pub fn gravity_at_altitude(altitude_m: f32) -> f32 {
    let term = 6371008.77 / (6371008.77 + altitude_m);
    GRAVITY * term * term
}

/// Create skew-symmetric matrix from vector (eq. 20)
/// This is used for cross product operations
pub fn skew_mat(v: &Vector3<f32>) -> Matrix3<f32> {
    Matrix3::new(
        0.0,    -v.z,    v.y,
        v.z,     0.0,   -v.x,
       -v.y,     v.x,    0.0,
    )
}

/// Outer product of a vector with itself: $$v v^T$$
pub fn vec_outer_product(v: &Vector3<f32>) -> Matrix3<f32> {
    v * v.transpose()
}

/// Convert rotation axis and angle to quaternion (eq. 101)
/// 
/// # Arguments
/// * `axis` - Normalized rotation axis
/// * `angle` - Rotation angle in radians
pub fn rot_axis2quaternion(axis: &Vector3<f32>, angle: f32) -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&na::Unit::new_normalize(*axis), angle)
}

/// Decompose a vector into magnitude and unit vector
pub fn decompose_vec(v: &Vector3<f32>) -> (f32, Vector3<f32>) {
    let mag = v.norm();
    
    if mag < 0.001 {
        defmt::debug!("Magnitude too small in decompose_vec, might get NaNs!");
    }
    
    let unit = if mag > 1e-9 {
        v / mag
    } else {
        Vector3::zeros()
    };
    
    (mag, unit)
}

/// Convert rotation vector to quaternion
/// Uses small-angle approximation for small magnitudes
pub fn vec2quaternion(v: &Vector3<f32>) -> UnitQuaternion<f32> {
    let mag = v.norm();
    
    if mag < 1e-9 {
        // Small-angle approximation
        let w = 1.0;
        let xyz = v * 0.5;
        return UnitQuaternion::from_quaternion(
            na::Quaternion::new(w, xyz.x, xyz.y, xyz.z)
        );
    }
    
    let axis = v / mag;
    rot_axis2quaternion(&axis, mag)
}

/// Convert rotation vector to rotation matrix (eq. 78)
/// Uses Rodrigues' rotation formula
pub fn vec2rot_matrix(v: &Vector3<f32>) -> Matrix3<f32> {
    let (mag, axis) = decompose_vec(v);
    
    let cos = cosf(mag);
    let sin = sinf(mag);
    
    let identity = Matrix3::identity();
    let skew = skew_mat(&axis);
    let outer = vec_outer_product(&axis);
    
    identity * cos + skew * sin + outer * (1.0 - cos)
}

/// Initialize the filter with IMU measurements
/// 
/// # Arguments
/// * `nominal` - Nominal state to initialize
/// * `accel_measurement` - Accelerometer reading (m/s²)
/// * `magnetometer_measurement` - Magnetometer reading
/// 
/// # Returns
/// Error code (OK or ERR_INIT_NO_ACCEL)
pub fn init_with_imu(
    nominal: &mut NominalState,
    accel_measurement: &Vector3<f32>,
    angular_vel_measurement: &Vector3<f32>,
    magnetometer_measurement: &Vector3<f32>,
) -> Result<(), FilterError> {
    let (accel_mag, accel_unit) = decompose_vec(accel_measurement);
    
    if accel_mag < 1e-3 {
        return Err(FilterError::InitNoAccel);
    }
    
    // Project magnetometer onto gravity vector
    let dot = accel_unit.dot(magnetometer_measurement);
    let proj = accel_unit * dot;
    
    // North is magnetometer minus its projection onto gravity
    let north_unnorm = magnetometer_measurement - proj;
    let (_, north_unit) = decompose_vec(&north_unnorm);
    
    // East is cross product of down (gravity) and north
    let east = accel_unit.cross(&north_unit);
    
    #[cfg(debug_assertions)]
    if DEBUG {
        let east_mag = east.norm();
        if fabsf(east_mag - 1.0) > 0.01 {
            defmt::error!("East vector should be a unit vector because it is the cross product of two orthogonal unit vectors, but it is not a unit vector!");
        }
    }
    
    // Build rotation matrix: columns are [-g, east, north]
    let rot_mat = Matrix3::from_columns(&[
        -accel_unit,
        east,
        north_unit,
    ]);
    
    nominal.rot_quaternion = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rot_mat));
    nominal.pos = Vector3::new(EARTH_RADIUS, 0.0, 0.0);
    
    Ok(())
}

/// Update state with IMU measurements
/// 
/// # Arguments
/// * `nominal` - Nominal state
/// * `error` - Error state
/// * `cov` - Covariance matrix
/// * `dt` - Time step (seconds)
/// * `accel_measurement` - Accelerometer reading (m/s²)
/// * `angular_vel_measurement` - Gyroscope reading (rad/s)
pub fn update_with_imu(
    nominal: &mut NominalState,
    error: &mut ErrorState,
    cov: &mut CovarianceMatrixP,
    dt: f32,
    accel_measurement: &Vector3<f32>,
    angular_vel_measurement: &Vector3<f32>,
) {
    defmt::debug!("Update with IMU: Starting nominal updates");
    
    // Section 5.4.1 - Nominal state updates
    let rot_mat = nominal.rot_quaternion.to_rotation_matrix();
    
    // Compute acceleration in world frame
    let accel_term = accel_measurement - nominal.accel_bias;
    let mut accel_world = rot_mat * accel_term;
    
    // Add gravity (pointing toward Earth center)
    let (pos_mag, pos_unit) = decompose_vec(&nominal.pos);
    let gravity = -pos_unit * GRAVITY;
    accel_world += gravity;
    nominal.accel = accel_world;
    
    // Update position (eq. 259a)
    nominal.pos += nominal.vel * dt + accel_world * (0.5 * dt * dt);
    
    // Update velocity (eq. 259b)
    nominal.vel += accel_world * dt;
    
    // Update quaternion (eq. 259c)
    let angular_vel = angular_vel_measurement - nominal.angular_vel_bias;
    let rotation_vec = angular_vel * dt;
    let delta_quat = vec2quaternion(&rotation_vec);
    nominal.rot_quaternion = nominal.rot_quaternion * delta_quat;
    nominal.rot_quaternion = UnitQuaternion::from_quaternion(nominal.rot_quaternion.into_inner()); // Renormalize
    
    defmt::debug!("Error updates");
    
    // Error state updates (eq. 260)
    
    // Update position error (eq. 260a)
    error.pos += error.vel * dt;
    
    // Prepare acceleration skew matrix for Jacobian
    let accel_body = accel_measurement - nominal.accel_bias;
    let accel_skew_body = skew_mat(&accel_body);
    let accel_skew_world = rot_mat.matrix() * accel_skew_body;
    let accel_mat_for_jacobian = -accel_skew_world * dt;
    
    // Update velocity error (eq. 260b)
    let mut vel_deterministic = accel_skew_world * error.angles_vector;
    vel_deterministic += rot_mat * error.accel_bias;
    vel_deterministic *= dt;
    error.vel -= vel_deterministic;
    
    // Prepare angular velocity matrix for Jacobian
    let angular_vel_term = (angular_vel_measurement - nominal.angular_vel_bias) * dt;
    let angles_rot_mat = vec2rot_matrix(&angular_vel_term);
    let angular_mat_for_jacobian = angles_rot_mat.transpose();
    
    // Update angles error (eq. 260c)
    error.angles_vector = angles_rot_mat * error.angles_vector;
    error.angles_vector -= error.angular_vel_bias * dt;
    
    defmt::debug!("Covariance matrix update");
    defmt::debug!("Set up Jacobian");
    
    // Update covariance matrix P (eq. 268)
    let mut jacobian_f = na::SMatrix::<f32, STATE_DIMS, STATE_DIMS>::zeros();
    
    defmt::debug!("Row 1");
    
    // Row 1: position
    jacobian_f.fixed_view_mut::<3, 3>(0, 0).copy_from(&Matrix3::identity());
    jacobian_f.fixed_view_mut::<3, 3>(0, 3).copy_from(&(Matrix3::identity() * dt));
    
    defmt::debug!("Row 2");
    
    // Row 2: velocity
    jacobian_f.fixed_view_mut::<3, 3>(3, 3).copy_from(&Matrix3::identity());
    jacobian_f.fixed_view_mut::<3, 3>(3, 6).copy_from(&accel_mat_for_jacobian);
    jacobian_f.fixed_view_mut::<3, 3>(3, 15).copy_from(&(Matrix3::identity() * dt));
    
    defmt::debug!("Row 3");
    
    // Row 3: angles
    jacobian_f.fixed_view_mut::<3, 3>(6, 6).copy_from(&angular_mat_for_jacobian);
    jacobian_f.fixed_view_mut::<3, 3>(6, 12).copy_from(&(Matrix3::identity() * -dt));
    
    defmt::debug!("Row 4");
    
    // Row 4: accel bias
    jacobian_f.fixed_view_mut::<3, 3>(9, 9).copy_from(&Matrix3::identity());
    
    defmt::debug!("Row 5");
    
    // Row 5: angular vel bias
    jacobian_f.fixed_view_mut::<3, 3>(12, 12).copy_from(&Matrix3::identity());
    
    defmt::debug!("Row 6");
    
    // Row 6: (reserved/unused in this implementation)
    jacobian_f.fixed_view_mut::<3, 3>(15, 15).copy_from(&Matrix3::identity());
    
    defmt::debug!("Multiply covariance by Jacobian of state pt. 1");
    
    // P = F * P * F^T
    let cov_copy = &jacobian_f * &cov.data;
    
    defmt::debug!("Multiply covariance by Jacobian of state pt. 2");
    defmt::debug!("Jacobian transpose");
    
    let jacobian_f_trans = jacobian_f.transpose();
    
    defmt::debug!("Multiply by Jacobian transpose");
    
    cov.data = cov_copy * jacobian_f_trans;
    
    defmt::debug!("Add covariance from uncertainty");
    
    // Add process noise Q
    let mut process_noise = na::SMatrix::<f32, STATE_DIMS, STATE_DIMS>::zeros();
    
    // Velocity noise
    for i in 0..3 {
        process_noise[(3 + i, 3 + i)] = VEL_NOISE;
    }
    
    // Angles noise
    for i in 0..3 {
        process_noise[(6 + i, 6 + i)] = ANGLES_NOISE;
    }
    
    // Accel bias noise
    for i in 0..3 {
        process_noise[(9 + i, 9 + i)] = ACCEL_NOISE;
    }
    
    // Angular vel bias noise
    for i in 0..3 {
        process_noise[(12 + i, 12 + i)] = ANGULAR_VEL_NOISE;
    }
    
    cov.data += process_noise;
}

/// Correct state from GPS measurements
/// 
/// # Arguments
/// * `nominal` - Nominal state
/// * `error` - Error state
/// * `cov` - Covariance matrix
/// * `gps_position` - GPS position measurement
/// * `gps_velocity` - GPS velocity measurement
pub fn correct_from_gps(
    nominal: &mut NominalState,
    error: &mut ErrorState,
    cov: &mut CovarianceMatrixP,
    gps_position: &Vector3<f32>,
    gps_velocity: &Vector3<f32>,
) {
    const N_MEAS: usize = 6;
    
    // GPS measurement noise covariance
    let h_acc = 1.5f32; // Horizontal accuracy
    let v_acc = 2.55f32; // Vertical accuracy
    let s_acc = 0.05f32; // Speed accuracy
    
    let mut v_mat = Matrix6::zeros();
    v_mat[(0, 0)] = h_acc * h_acc;
    v_mat[(1, 1)] = h_acc * h_acc;
    v_mat[(2, 2)] = v_acc * v_acc;
    v_mat[(3, 3)] = s_acc * s_acc;
    v_mat[(4, 4)] = s_acc * s_acc;
    v_mat[(5, 5)] = s_acc * s_acc;
    
    // Kalman gain: K = P * H^T * (H * P * H^T + V)^-1
    // H is [I_6x6, 0_6x12] for GPS (measures position and velocity)
    
    // H * P * H^T is just the top-left 6x6 block of P
    let mut s_mat = v_mat.clone();
    s_mat += cov.data.fixed_view::<6, 6>(0, 0);
    
    // Invert S
    let s_inv = match s_mat.try_inverse() {
        Some(inv) => inv,
        None => {
            defmt::info!("GPS update: Matrix inversion failed!");
            return;
        }
    };
    
    // P * H^T is the first 6 columns of P
    let pht = cov.data.fixed_view::<STATE_DIMS, 6>(0, 0);
    
    // K = P * H^T * S^-1
    let kalman_gain = pht * s_inv;
    
    // Update covariance using Joseph form (eq. 275)
    // P = (I - K*H) * P * (I - K*H)^T + K * V * K^T
    
    let mut ikh = na::SMatrix::<f32, STATE_DIMS, STATE_DIMS>::identity();
    for i in 0..N_MEAS {
        for j in 0..STATE_DIMS {
            ikh[(i, j)] -= kalman_gain[(j, i)];
        }
    }
    
    let temp = &ikh * &cov.data * ikh.transpose();
    let kvkt = &kalman_gain * &v_mat * kalman_gain.transpose();
    
    cov.data = temp + kvkt;
    
    // Note: Actual state correction would go here
    // (innovation calculation and state update)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_skew_mat() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let skew = skew_mat(&v);
        
        assert_eq!(skew[(0, 0)], 0.0);
        assert_eq!(skew[(0, 1)], -3.0);
        assert_eq!(skew[(0, 2)], 2.0);
    }
    
    #[test]
    fn test_decompose_vec() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        let (mag, unit) = decompose_vec(&v);
        
        assert!((mag - 5.0).abs() < 1e-6);
        assert!((unit.norm() - 1.0).abs() < 1e-6);
    }
}