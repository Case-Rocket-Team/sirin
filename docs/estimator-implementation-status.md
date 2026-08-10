# Estimator implementation status

## Implemented baseline

- Strict 15-state right-error ESKF with centralized state indices.
- Nonzero, configurable initial covariance.
- Midpoint nominal IMU integration in local NED.
- Continuous F/G/Qc covariance model and second-order state transition.
- Generic fixed-size Cholesky Kalman update.
- NIS rejection, Joseph covariance update, injection, and reset Jacobian.
- Stationary tilt and gyro-bias initializer.
- GPS position and native NED velocity measurement models.
- NAV-PVT velocity, speed accuracy, fix quality, and satellite population.
- Relative barometric-height and full-vector magnetometer models.
- WGS-84 geodetic/ECEF/local-NED conversion.
- Physical timestamp, sequence, saturation, stale, and future checks.
- Deterministic estimator event processor and diagnostic counters.
- Bounded fixed-lag GNSS rewind/repropagation with replay tests.
- Explicit shared sensor and navigation-solution contracts.
- Static sensor calibration transforms and aiding-source hysteresis health.
- Host-side eskf-sim example with deterministic delayed-GPS assertions.

## Verification status

The repository passes git diff --check. Rust compilation and unit tests have
not been run in this environment because no Rust toolchain is available, and
no toolchain should be installed here.

Run these checks in an existing Rust development environment:

~~~text
cargo test -p sirin-filter --target x86_64-pc-windows-msvc
cargo check -p sirin-filter --target thumbv7em-none-eabihf
cargo check -p sirin --target thumbv7em-none-eabihf
~~~

The first compile should focus on nalgebra const-generic inference and the
u-blox NAV-PVT accessor return types. The intended receiver accessor units are
meters per second; the shared packet converts them to compact centimeters per
second and the ESKF adapter converts back to SI.

## Highest-priority remaining work

1. Wire a dedicated IMU-driven Embassy task to EstimatorRuntime. Do not run
   propagation from the existing 100 or 500 millisecond application loops.
2. Timestamp IMU capture at data-ready/FIFO read time and assign real sequence
   counters and saturation masks.
3. Convert GNSS position to local NED using a prelaunch averaged origin.
4. Log serialized estimator events, update results, and covariance diagonals.
5. Add configured sensor-to-body rotations and GNSS antenna lever arm.
6. Measure IMU noise and replace every placeholder in EskfConfig.
7. Add barometer reference initialization, then a barometer-bias state.
8. Add flight-phase health policy before using estimates for control.

## Safety limitations

This implementation is a development baseline. It has not been target-built,
statistically tuned, replay-tested, or flight-qualified. Do not connect it to
deployment or closed-loop control until the gates in the reviewed plan have
been completed.
