# sirin-filter

sirin-filter is the allocation-free, no_std 15-state error-state Kalman filter
used by Sirin.

The filter state is local North-East-Down. Sensor drivers must apply their
calibrated sensor-to-body rotation and convert to SI units before constructing
an observation. See docs/estimator-conventions.md for the complete contract.

## Minimal use

~~~rust
use sirin_filter::{Eskf, ImuSample, Vec3};

let mut filter = Eskf::default();
filter.propagate_imu(ImuSample {
    timestamp_us: 1_000_000,
    accel_mps2_b: Vec3::new(0.0, 0.0, -9.80665),
    gyro_radps_b: Vec3::zeros(),
    temperature_c: None,
    accel_saturated: [false; 3],
    gyro_saturated: [false; 3],
    sequence: 0,
})?;
~~~

Use StationaryInitializer before flight to estimate initial tilt and gyro
bias. Establish a LocalNedOrigin from a good prelaunch GNSS fix, convert GNSS
positions at the runtime boundary, then call the velocity and position fusion
methods with the receiver-provided variances.

The numerical defaults are conservative placeholders. IMU noise, magnetic
field, measurement variance floors, gates, sensor-to-body rotations, and
lever arms must be replaced with measured vehicle configuration before flight.

## Host simulation

The workspace example examples/eskf-sim runs a deterministic 5-second
trajectory with delayed GPS position and velocity updates:

~~~text
cargo test -p eskf-sim
cargo run -p eskf-sim
~~~

## Current scope

Implemented:

- Midpoint IMU nominal-state propagation.
- 15-state covariance propagation.
- GPS position and velocity updates.
- Relative barometric-height update.
- Calibrated full-vector magnetometer update.
- Joseph covariance update, NIS gating, error injection, and reset.
- Stationary initialization and WGS-84 local-NED conversion.
- Physical timestamp, sequence, saturation, and stale/future measurement checks.
- Bounded fixed-lag GNSS rewind/repropagation.
- Static calibration transforms and aiding-source hysteresis health.

Not yet implemented:

- Flight-phase health/recovery policy.
- GNSS and sensor lever-arm compensation.
- Barometer-bias state.
- Firmware estimator task and deterministic log replay.
