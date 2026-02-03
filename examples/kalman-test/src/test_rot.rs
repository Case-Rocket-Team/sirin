//! GPT 5.2-generated code to test that rotations are accurate.W

use core::f32::consts::PI;
use defmt::info;
use embassy_time::{Duration, Instant};
use micromath::F32Ext;
use sirin::{Sirin, state::*, uunit::WithUnits};

// Whatever your units wrapper is:

fn quat_to_roll_deg(n: &NominalState) -> f32 {
    // Assumes nominal has a quaternion field; adjust names/order to your struct.
    // Common layouts: (w,x,y,z) or (x,y,z,w). Fix to match yours.
    let q = &n.rot_quaternion; // <- change to your actual field, e.g. n.attitude.q, n.quat, etc.
    let (r, x, y, z) = (q.r, q.x, q.y, q.z);

    // Roll (x-axis rotation), in radians:
    let sinr_cosp = 2.0 * (r * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    sinr_cosp.atan2(cosr_cosp) * 180.0 / PI
}

fn wrap180(mut deg: f32) -> f32 {
    // map to [-180, 180)
    deg = (deg + 180.0) % 360.0;
    if deg < 0.0 { deg += 360.0; }
    deg - 180.0
}

/// Simulate an IMU performing a 180° roll over `duration_s`.
/// - Gyro: constant roll rate
/// - Accel: gravity vector rotated consistently with the roll angle (no linear accel)
pub async fn test_180deg_roll_kalman(sirin: &mut Sirin) {
    // --- Filter state ---
    let mut nominal = NominalState::default();
    let mut error = ErrorState::default();
    let mut cov = CovarianceMatrixP::default();

    // --- Simulation parameters ---
    let duration_s: f32 = 4.0;       // complete 180° in 4 seconds
    let rate_deg_s: f32 = 180.0 / duration_s;
    let rate_rad_s: f32 = rate_deg_s * PI / 180.0;

    let dt = 0.01f32;                // 100 Hz sim
    let steps = (duration_s / dt).round() as usize;

    // Start aligned: roll=0 => accel ~ [0,0,-g] or [0,0,+g] depending on convention.
    // Pick one and keep it consistent with your filter. Here we use +Z up? adjust if needed.
    let g = 9.81f32;

    // Initialize filter with the first reading
    {
        let roll0 = 0.0f32;
        let accel0 = [
            0.0,
            g * roll0.sin(),          // y component
            g * roll0.cos(),          // z component
        ];
        let gyro0 = [rate_rad_s, 0.0, 0.0]; // rolling about X

        unsafe {
            sirin_c::init_with_imu(
                &mut nominal,
                &mut error,
                accel0.as_ptr(),
                gyro0.as_ptr(),
            );
        }
    }

    // --- Run simulation ---
    let mut roll_true = 0.0f32;

    for k in 0..steps {
        roll_true = (k as f32 + 1.0) * dt * rate_rad_s; // radians, monotonic to PI

        // Gyro: constant roll rate around X
        let angular = [rate_rad_s, 0.0, 0.0];

        // Accel: gravity rotated by roll_true (no other accel).
        // If your IMU "at rest" reads +g on +Z, keep it. If it reads -g, flip sign.
        let accel = [
            0.0,
            g * roll_true.sin(),
            g * roll_true.cos(),
        ];

        unsafe {
            sirin_c::update_with_imu(
                &mut nominal,
                &mut error,
                &mut cov,
                dt.with_units(),
                accel.as_ptr(),
                angular.as_ptr(),
            );
        }

        // Occasionally print progress
        if k % 50 == 0 || k + 1 == steps {
            let est_roll = wrap180(quat_to_roll_deg(&nominal));
            let true_roll = wrap180(roll_true * 180.0 / PI);
            info!(
                "k={} true_roll={} deg, est_roll={} deg",
                k, true_roll, est_roll
            );
        }
    }

    // --- Check result ---
    let est_roll = wrap180(quat_to_roll_deg(&nominal));
    let true_roll = wrap180(180.0);

    let err = wrap180(est_roll - true_roll).abs();
    let tol_deg = 5.0; // pick a tolerance appropriate for your filter tuning
    assert!(
        err <= tol_deg,
        "Expected ~180° roll. Got {:.2}° (err {:.2}°)",
        est_roll,
        err
    );
}
