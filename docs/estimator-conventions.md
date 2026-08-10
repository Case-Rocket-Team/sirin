# Estimator conventions

This document is the contract for the Sirin navigation estimator. Equations,
drivers, calibration data, telemetry, and tests must use these conventions.

The authoritative implementation is the Rust sirin-filter crate. The sirin-c
filter entry points are legacy and must not be extended with new estimator
features.

## Frames

- The navigation frame is a fixed local North-East-Down (`NED`) frame whose
  origin is established from a good prelaunch GNSS fix.
- Navigation vectors are ordered `[north, east, down]`. Gravity is therefore
  `[0, 0, +g]` and local height is `-position_ned.z`.
- The body frame is the calibrated vehicle/PCB frame. Its exact physical axes
  still need to be recorded from the PCB drawing and verified on hardware.
- `R_NB` rotates a body-frame vector into NED: `v_N = R_NB * v_B`.
- The filter position is the IMU sensing origin. Other sensor locations are
  represented by configured body-frame lever arms from the IMU.

## Attitude and error state

- Quaternions use nalgebra's Hamilton convention and scalar-first constructor
  order `(w, x, y, z)`.
- The nominal quaternion represents `R_NB`.
- Quaternion composition is applied on the right for a body-frame increment:
  `q_next = q_NB * Exp(omega_B * dt)`.
- The attitude error is right multiplicative:
  `R_true_NB = R_nominal_NB * Exp([delta_theta_B]x)`.
- Corrections are injected with
  `q_NB <- q_NB * Exp(delta_theta_B)`.
- Measurement residuals are `measurement - prediction`.
- After injection, the error mean is reset to zero and covariance is
  transformed with the right-error reset Jacobian.

## Core state

The nominal state is position and velocity in NED, body-to-NED attitude,
accelerometer bias in body coordinates, and gyro bias in body coordinates.

The 15-element error state is ordered exactly as follows:

| Range | State |
| --- | --- |
| `0..3` | NED position error |
| `3..6` | NED velocity error |
| `6..9` | body-frame attitude error |
| `9..12` | body-frame accelerometer-bias error |
| `12..15` | body-frame gyro-bias error |

## Units and time

- Filter scalar values are `f32` on the target.
- Angles are radians and angular rates are radians/second.
- Position, velocity, acceleration, and specific force use SI units.
- Magnetic field observations use microtesla.
- Pressure observations use pascals.
- Estimator time is monotonic `u64` microseconds since boot.
- A sample timestamp describes the physical capture/conversion epoch, not when
  a driver finishes delivering it. Arrival time may be logged separately.
- GNSS geodetic conversion uses ellipsoid height. MSL height remains a distinct
  field and must never be substituted silently.

## Decisions still requiring hardware verification

Before estimator output is used for control, document and test:

1. PCB/body positive X, Y, and Z directions.
2. Sensor-to-body rotations for every populated sensor.
3. IMU, GNSS antenna, magnetometer, and barometer-port locations.
4. GNSS height datum and receiver measurement latency.
5. Expected flight duration, range, altitude, angular rate, and acceleration.
6. Acceptable output accuracy and latency for each flight phase.
