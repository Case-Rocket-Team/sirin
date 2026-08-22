# ESKF Integration - Concrete Code Changes

This document shows the exact code to add to `examples/nosecone-ebay/src/main.rs`.

## Prerequisites
Add to imports (around line 17):
```rust
use sirin_filter::{
    Eskf, EskfConfig, InitialUncertainty, ImuNoise,
    FixedLagHistory, HistoryProcessResult,
    ImuSample, BarometerObservation, GpsPositionObservation, GpsVelocityObservation,
    MagnetometerObservation, Vec3, EstimatorDiagnostics,
};
```

---

## Phase 1: Configuration & Initialization

### Step 1A: Add Helper Functions (beginning of main_task)

```rust
/// Initialize filter attitude from accelerometer + magnetometer
/// Returns nominal state with attitude aligned to gravity and north
fn initialize_from_sensors(
    accel_mps2: Vec3,
    mag_ut: Vec3,
) -> Result<sirin_filter::NominalState, &'static str> {
    use nalgebra::UnitQuaternion;
    
    // Normalize accelerometer to get gravity direction (points up)
    let accel_norm = accel_mps2.norm();
    if accel_norm < 1.0 {
        return Err("Acceleration magnitude too small for initialization");
    }
    let down = -accel_mps2 / accel_norm;  // Negative: gravity points down
    
    // Normalize magnetometer
    let mag_norm = mag_ut.norm();
    if mag_norm < 10000.0 {  // < 10 μT
        return Err("Magnetic field magnitude too small");
    }
    let mag_unit = mag_ut / mag_norm;
    
    // Project mag onto horizontal plane to get north
    let down_component = mag_unit.dot(&down);
    let mag_horizontal = mag_unit - down * down_component;
    let mag_horizontal_norm = mag_horizontal.norm();
    
    if mag_horizontal_norm < 0.1 {
        return Err("Horizontal magnetic field component too small");
    }
    
    let north = mag_horizontal / mag_horizontal_norm;
    
    // East is cross product of down and north
    // Down(up vector) × North → East (right-hand rule)
    let up = -down;
    let east = north.cross(&up);
    
    // Build rotation matrix [north, east, down] = rotation matrix from NED to body
    // So R_ned_to_body^T is what we want (body to NED)
    let r_ned_to_body = nalgebra::Matrix3::from_columns(&[north, east, down]);
    let r_body_to_ned = r_ned_to_body.transpose();
    
    let rot_ned_to_body = nalgebra::Rotation3::from_matrix_unchecked(r_ned_to_body);
    let attitude_body_to_ned = UnitQuaternion::from_rotation_matrix(&rot_body_to_ned);
    
    Ok(sirin_filter::NominalState {
        pos: Vec3::zeros(),  // Start at origin
        vel: Vec3::zeros(),  // Start at rest
        rot_quaternion: attitude_body_to_ned,
        accel_bias_mps2: Vec3::zeros(),  // Will be estimated
        gyro_bias_radps: Vec3::zeros(),  // Will be estimated
    })
}

/// Convert micro-Gs to m/s²
fn microgs_to_mps2(accel_microgs: Vec3) -> Vec3 {
    Vec3::new(
        accel_microgs.x * 1e-6 * 9.80665,
        accel_microgs.y * 1e-6 * 9.80665,
        accel_microgs.z * 1e-6 * 9.80665,
    )
}

/// Convert micro-Deg/s to rad/s
fn microdegps_to_radps(gyro_microdegps: Vec3) -> Vec3 {
    const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;
    Vec3::new(
        gyro_microdegps.x * 1e-6 * DEG_TO_RAD,
        gyro_microdegps.y * 1e-6 * DEG_TO_RAD,
        gyro_microdegps.z * 1e-6 * DEG_TO_RAD,
    )
}
```

### Step 1B: Create Filter Configuration (after initial_altitude setup, ~line 99)

```rust
    //============================================================================
    // Initialize Error-State Kalman Filter (ESKF)
    //============================================================================
    
    // Define filter configuration for your location/hardware
    let filter_config = EskfConfig {
        initial_uncertainty: InitialUncertainty {
            position_m: 100.0,           // Large initial position uncertainty
            velocity_mps: 20.0,           // Unknown launch velocity
            attitude_rad: 0.2,            // ~11 degrees initial attitude uncertainty
            accel_bias_mps2: 0.5,         // ±0.5 m/s² accelerometer bias
            gyro_bias_radps: 0.01,        // ±0.01 rad/s gyroscope bias
        },
        imu_noise: ImuNoise {
            accel_white_noise_mps2: 0.002158,   // 220 mg√Hz from sensor specs
            gyro_white_noise_radps: 0.00006632, // 3.8 mdps√Hz from sensor specs
            accel_random_walk: 6.20e-6,         // Bias random walk (m/s²/√Hz)
            gyro_random_walk: 2.76e-7,          // Bias random walk (rad/s/√Hz)
        },
        // Magnetic field for Cleveland, OH (update for your location!)
        // Get from: https://www.ngdc.noaa.gov/geomag/declination
        magnetic_field_ned_ut: [
            19000.0,   // North component (μT)
            -2700.0,   // East component (μT)  
            48000.0,   // Down component (μT)
        ],
        // Measurement gate thresholds (reject if innovation > gate * σ)
        barometer_gate: 3.0,
        gps_position_gate: 3.0,
        gps_velocity_gate: 3.0,
        magnetometer_gate: 3.0,
        // Measurement timing constraints
        max_measurement_age_us: 5_000_000,          // Reject measurements > 5 seconds old
        future_measurement_tolerance_us: 100_000,   // Allow 100ms into future (clock sync margin)
    };
    
    info!("Filter config initialized");
    
    // Read initial IMU/magnetometer samples for attitude initialization
    let init_accel = sirin.imu.read().await?.accel;
    let init_mag = sirin.magnetometer.read().await?.mag;
    
    let initial_nominal_state = initialize_from_sensors(
        microgs_to_mps2(Vec3::new(
            init_accel.x as f32,
            init_accel.y as f32,
            init_accel.z as f32,
        )),
        Vec3::new(
            init_mag.x as f32,
            init_mag.y as f32,
            init_mag.z as f32,
        ),
    ).expect("Failed to initialize attitude from sensors");
    
    info!("Initial attitude computed: q={:?}", initial_nominal_state.rot_quaternion);
    
    // Create filter with initial state and covariance
    let initial_covariance = sirin_filter::CovarianceMatrixP::from_initial_uncertainty(
        &filter_config.initial_uncertainty
    );
    
    let mut filter = Eskf::new(
        initial_nominal_state,
        initial_covariance,
        filter_config,
    );
    
    // Create fixed-lag history smoother for delayed measurements
    // History buffer: 64 samples at ~10 Hz = ~6.4 seconds of lag
    let mut history = FixedLagHistory::<64>::new(filter.clone());
    
    info!("ESKF initialized successfully");
```

---

## Phase 2: IMU Integration

### Step 2A: Add Timestamp Tracking (after `let mut i = 0;` around line 122)

```rust
    let mut current_time_us: u64 = 0;
    let mut last_filter_time_us: u64 = 0;
```

### Step 2B: Update Timestamps in Main Loop (in the loop, around line 237 after `ticker.next().await;`)

```rust
        // Update current time
        if let Some(epoch) = duration_since_epoch() {
            current_time_us = epoch.as_millis() as u64 * 1000;  // ms to μs
        } else {
            current_time_us = current_time_us.saturating_add(100_000);  // Fallback: +100ms
        }
```

### Step 2C: Replace Old Filter Comment with New Code (replace line 248)

```rust
        //====================================================================
        // ESKF Propagation with IMU
        //====================================================================
        
        if let Ok(imu) = &sirin.data.imu.accel {
            let imu_sample = ImuSample {
                timestamp_us: current_time_us,
                // Convert micro-Gs to m/s²
                accel_mps2_b: microgs_to_mps2(Vec3::new(
                    sirin.data.imu.accel.as_ref().ok().map(|a| a.x as f32).unwrap_or(0.0),
                    sirin.data.imu.accel.as_ref().ok().map(|a| a.y as f32).unwrap_or(0.0),
                    sirin.data.imu.accel.as_ref().ok().map(|a| a.z as f32).unwrap_or(0.0),
                )),
                // Convert micro-deg/s to rad/s
                gyro_radps_b: microdegps_to_radps(Vec3::new(
                    sirin.data.imu.angular_vel.as_ref().ok().map(|g| g.x as f32).unwrap_or(0.0),
                    sirin.data.imu.angular_vel.as_ref().ok().map(|g| g.y as f32).unwrap_or(0.0),
                    sirin.data.imu.angular_vel.as_ref().ok().map(|g| g.z as f32).unwrap_or(0.0),
                )),
                temperature_c: None,
                accel_saturated: [false; 3],  // TODO: get from sensor status
                gyro_saturated: [false; 3],
                sequence: i as u32,
            };
            
            match filter.propagate_imu(imu_sample) {
                Ok(()) => {
                    // IMU propagation succeeded
                    last_filter_time_us = current_time_us;
                },
                Err(e) => {
                    warn!("IMU propagation error: {:?}", e);
                }
            }
            
            // Also process through history for delayed measurements
            let _ = history.process_imu(imu_sample);
        }
        
        // Extract filter estimates for state and logging
        let nav_solution = filter.navigation_solution();
        state.nominal.pos = Vec3::new(
            nav_solution.position_ned_m[0],
            nav_solution.position_ned_m[1],
            nav_solution.position_ned_m[2],
        );
        state.nominal.vel = Vec3::new(
            nav_solution.velocity_ned_mps[0],
            nav_solution.velocity_ned_mps[1],
            nav_solution.velocity_ned_mps[2],
        );
        state.nominal.rot_quaternion = na::UnitQuaternion::from_quaternion(
            na::Quaternion::new(
                nav_solution.attitude_nb_wxyz[0],  // w
                nav_solution.attitude_nb_wxyz[1],  // x
                nav_solution.attitude_nb_wxyz[2],  // y
                nav_solution.attitude_nb_wxyz[3],  // z
            )
        );
```

---

## Phase 3: Barometer Integration

### Step 3A: Add Barometer Fusion (after IMU block, before DataState calculation)

```rust
        //====================================================================
        // Barometer Altitude Update
        //====================================================================
        
        if let Ok(pressure) = sirin.data.baro.pressure {
            // Convert absolute pressure to height above sea level
            let measured_altitude = approx_pressure_altitude(pressure.convert());
            let height_up_m = (measured_altitude - initial_altitude).value as f32;
            
            let baro_obs = BarometerObservation {
                timestamp_us: current_time_us,
                height_up_m,
                variance_m2: 0.25,  // ~0.5 m standard deviation
            };
            
            match filter.fuse_barometer(&baro_obs) {
                Ok(result) => {
                    if result.accepted {
                        debug!("Barometer accepted: {:.1}m", height_up_m);
                    } else {
                        debug!("Barometer rejected (gate): {:.1}m", height_up_m);
                    }
                },
                Err(e) => {
                    warn!("Barometer fusion error: {:?}", e);
                }
            }
        }
```

---

## Validation Commands

### Ground Testing
```bash
# Build for native target
cd examples/eskf-sim && cargo build --bin eskf-sim

# Run simulation to verify math
cargo run --bin eskf-sim

# Build firmware for hardware
cd ../nosecone-ebay && cargo build
```

### Debugging on Hardware
Add to main loop to log filter diagnostics:
```rust
if i % 10 == 0 {  // Every 1 second
    let diag = filter.diagnostics();
    info!("Filter covariance trace: {:.3}", diag.covariance_trace);
    info!("Position NED: [{:.1}, {:.1}, {:.1}]", 
        state.nominal.pos.x, state.nominal.pos.y, state.nominal.pos.z);
    info!("Velocity NED: [{:.2}, {:.2}, {:.2}]",
        state.nominal.vel.x, state.nominal.vel.y, state.nominal.vel.z);
}
```

---

## Key Integration Points

| Phase | Location | Code Size | Dependencies |
|-------|----------|-----------|--------------|
| 1: Init | Line ~99 | ~200 LOC | Config setup only |
| 2: IMU | Line ~248 | ~100 LOC | Helper functions from Phase 1 |
| 3: Baro | After IMU | ~20 LOC | Filter propagation working |
| 4: GPS | TODO | ~50 LOC | Coordinate transformation needed |
| 5: Mag | TODO | ~25 LOC | Reference field configured |

---

## Common Pitfalls

1. **Unit conversions wrong**: micro-Gs and micro-Deg/s are VERY small (1e-6 scale)
2. **Timestamp monotonicity**: filter will reject measurements with non-increasing timestamps
3. **NED frame confusion**: North-East-Down, NOT Earth-Centered-Earth-Fixed
4. **Quaternion order**: WXYZ not XYZW - check carefully!
5. **Magnetic field**: Must match actual location, not default values

