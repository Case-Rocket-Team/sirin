# ESKF Integration Plan for nosecone-ebay

## Goal
Replace the old partial filter implementation with the new Error-State Kalman Filter (ESKF) to properly validate state estimation math on hardware.

## Current State
- `examples/nosecone-ebay/src/main.rs` has state structure with `nominal.pos`, `nominal.vel`, `nominal.rot_quaternion`
- Filter integration is commented out (line 248)
- Only barometric altitude is being calculated; position/velocity/attitude estimates are not updated
- IMU, GPS, barometer, and magnetometer data is collected but not fused

## Target State
- ESKF propagates IMU data every 100ms (10 Hz)
- GPS position/velocity updates fused when available (~1 Hz)
- Barometer height updates fused every 100ms
- Magnetometer updates fused periodically
- State estimates (pos, vel, attitude) properly converge to truth

---

## Phase 1: Configuration & Initialization (Week 1)

### 1.1 Define Filter Configuration
**File:** `examples/nosecone-ebay/src/main.rs` - Add at top of `main_task()`

Create proper `EskfConfig` struct:
```rust
// Sensor calibration for your location & sensors
let filter_config = EskfConfig {
    initial_uncertainty: InitialUncertainty {
        position_m: 100.0,           // Large initial uncertainty
        velocity_mps: 20.0,           // Unknown launch velocity
        attitude_rad: 0.2,            // ~11 degrees
        accel_bias_mps2: 0.5,
        gyro_bias_radps: 0.01,
    },
    imu_noise: ImuNoise {
        accel_white_noise_mps2: 0.002158,   // From hardware specs
        gyro_white_noise_radps: 0.00006632,
        accel_random_walk: 6.20e-6,
        gyro_random_walk: 2.76e-7,
    },
    magnetic_field_ned_ut: [
        19000.0,   // North component (μT) - update for Cleveland location
        -2700.0,   // East component (μT)
        48000.0,   // Down component (μT)
    ],
    barometer_gate: 3.0,
    gps_position_gate: 3.0,
    gps_velocity_gate: 3.0,
    magnetometer_gate: 3.0,
    max_measurement_age_us: 5_000_000,  // 5 seconds
    future_measurement_tolerance_us: 100_000,  // 100ms
};
```

**Validation:**
- [ ] Verify magnetic field components match actual location
- [ ] Check sensor noise specs from datasheets
- [ ] Confirm gate thresholds are reasonable (typically 2-4 σ)

---

### 1.2 Create Filter Initialization
**File:** `examples/nosecone-ebay/src/main.rs` - After initial_altitude setup (line 98)

```rust
// Initialize filter state
let mut filter = {
    // Initial attitude: level, pointing north
    // Use accelerometer + magnetometer to estimate
    let accel = sirin.imu.read().await?.accel;
    let mag = sirin.magnetometer.read().await?.mag;
    
    let nominal_state = initialize_from_sensors(
        accel.convert_to_mps2(),
        mag.convert_to_microTesla(),
    ).expect("Failed to initialize filter");
    
    let covariance = CovarianceMatrixP::from_initial_uncertainty(&filter_config.initial_uncertainty);
    
    sirin_filter::Eskf::new(nominal_state, covariance, filter_config)
};

// Create history for delayed measurements (GPS, barometer)
let mut history = FixedLagHistory::<64>::default();
```

**Key Decisions:**
- Initial position: Start at NED origin (reference point)
- Initial velocity: Zero (on launch pad)
- Initial attitude: Computed from accel + magnetometer
- History lag: 64 samples @ 10Hz = 6.4 seconds (enough for GPS/baro delay)

**Validation:**
- [ ] Filter initializes without panicking
- [ ] Initial attitude looks reasonable (verify with prints)
- [ ] Covariance matrix is positive definite

---

## Phase 2: IMU Integration (Week 1)

### 2.1 Add IMU Measurement Loop
**File:** `examples/nosecone-ebay/src/main.rs` - Replace line 248

```rust
// Process IMU through Kalman filter
if let Ok(imu) = &sirin.data.imu {
    let imu_sample = ImuSample {
        timestamp_us: current_time_us,
        accel_mps2_b: imu.accel.convert_to_mps2(),  // Convert micro-Gs to m/s²
        gyro_radps_b: imu.angular_vel.convert_to_radps(),  // Convert micro-deg/s to rad/s
        temperature_c: None,
        accel_saturated: [false; 3],  // Update from sensor status
        gyro_saturated: [false; 3],
        sequence: i as u32,
    };
    
    if let Err(e) = filter.propagate_imu(imu_sample) {
        warn!("IMU propagation failed: {:?}", e);
    } else if let Err(e) = history.process_imu(imu_sample) {
        warn!("History processing failed: {:?}", e);
    }
}

// Extract estimates for state logging
let nav_solution = filter.navigation_solution();
state.nominal.pos = Vec3::new(nav_solution.position_ned_m[0], nav_solution.position_ned_m[1], nav_solution.position_ned_m[2]);
state.nominal.vel = Vec3::new(nav_solution.velocity_ned_mps[0], nav_solution.velocity_ned_mps[1], nav_solution.velocity_ned_mps[2]);
state.nominal.rot_quaternion = UnitQuaternion::from_quaternion(na::Quaternion::new(
    nav_solution.attitude_nb_wxyz[0],
    nav_solution.attitude_nb_wxyz[1],
    nav_solution.attitude_nb_wxyz[2],
    nav_solution.attitude_nb_wxyz[3],
));
```

**Coordinate Conversions Needed:**
- micro-Gs → m/s²: multiply by `1e-6 * 9.80665`
- micro-deg/s → rad/s: multiply by `1e-6 * π/180`

**Validation:**
- [ ] IMU propagation succeeds every 100ms
- [ ] State updates smoothly (no jumps)
- [ ] Attitude rotates when tilting (test by hand)
- [ ] Velocity integrates from acceleration

---

## Phase 3: Measurement Fusion (Week 2)

### 3.1 Barometer Integration
**File:** `examples/nosecone-ebay/src/main.rs` - After IMU processing

```rust
if let Ok(pressure) = sirin.data.baro.pressure {
    // Convert pressure to height above launch site
    let measured_altitude = approx_pressure_altitude(pressure.convert());
    let height_up_m = (measured_altitude - initial_altitude).value as f32;
    
    let baro_obs = BarometerObservation {
        timestamp_us: current_time_us,
        height_up_m,
        variance_m2: 0.25,  // ~0.5m std dev
    };
    
    let result = filter.fuse_barometer(&baro_obs);
    if result.accepted {
        info!("Baro accepted: {:.1}m", height_up_m);
    }
}
```

**Validation:**
- [ ] Barometer measurements accepted
- [ ] Height estimate matches pressure altitude
- [ ] No divergence in position Z after fusion

### 3.2 GPS Integration  
**File:** `examples/nosecone-ebay/src/main.rs` - In main loop

```rust
if let Some(gps_fix) = &sirin.data.gps_fix {
    if gps_fix.fix_type != GpsFixType::NoFix {
        // Convert ECEF/LLA to NED relative to launch site
        let pos_ned = gps_to_ned(gps_fix, launch_site_reference);
        let vel_ned = gps_velocity_to_ned(gps_fix);
        
        let pos_obs = GpsPositionObservation {
            timestamp_us: gps_fix.timestamp_us,
            position_ned_m: pos_ned,
            variance_m2: Vec3::repeat((gps_fix.pos_acc as f32 / 1000.0).powi(2)),
        };
        
        let vel_obs = GpsVelocityObservation {
            timestamp_us: gps_fix.timestamp_us,
            velocity_ned_mps: vel_ned,
            variance_m2ps2: Vec3::repeat((gps_fix.vel_acc as f32 / 1000.0).powi(2)),
        };
        
        // Use fixed-lag history for delayed measurements
        if let HistoryProcessResult::Measurement(result) = history.fuse_gps_position(&pos_obs) {
            if result.accepted {
                info!("GPS pos accepted, NED={:?}", pos_ned);
            }
        }
        if let HistoryProcessResult::Measurement(result) = history.fuse_gps_velocity(&vel_obs) {
            if result.accepted {
                info!("GPS vel accepted, NED={:?}", vel_ned);
            }
        }
    }
}
```

**Critical:** Define coordinate conversion functions:
```rust
fn gps_to_ned(fix: &GpsFix, reference: &NedReference) -> Vec3 {
    // Convert GPS lat/lon/alt to NED from reference point
    // Implementation depends on your coordinate system
}
```

**Validation:**
- [ ] GPS position measurements accepted
- [ ] Position estimates converge to GPS values
- [ ] Velocity estimates converge to GPS velocity

### 3.3 Magnetometer Integration
**File:** `examples/nosecone-ebay/src/main.rs` - In main loop

```rust
if let Ok(mag) = &sirin.data.magnetometer.mag {
    let mag_obs = MagnetometerObservation {
        timestamp_us: current_time_us,
        field_ut_b: Vec3::new(
            mag.x as f32,
            mag.y as f32,
            mag.z as f32,
        ),
        variance_ut2: Vec3::repeat(100.0),  // ~10 μT std dev per axis
    };
    
    let result = filter.fuse_magnetometer(&mag_obs);
    if result.accepted {
        info!("Mag accepted");
    }
}
```

**Validation:**
- [ ] Magnetometer measurements accepted
- [ ] Yaw converges to correct heading
- [ ] No oscillation or divergence

---

## Phase 4: Testing & Validation (Week 2-3)

### 4.1 Ground Testing Checklist
Before flying:

**Stationary Test (30 minutes on bench):**
- [ ] Filter initializes without errors
- [ ] Attitude estimate is stable (< 0.1 rad change when stationary)
- [ ] Position/velocity estimates stay near zero
- [ ] No covariance divergence (NaN/Inf)
- [ ] All measurement gates working (accept/reject decisions make sense)

**Motion Test (handheld motion):**
- [ ] Tilting phone rotates attitude estimate (pitch/roll)
- [ ] Rotating phone changes yaw
- [ ] Velocity integrates from acceleration
- [ ] Velocity decays after motion stops
- [ ] No attitude flips or sign ambiguity

**GPS Test (if available indoors with strong signal):**
- [ ] GPS position is accepted
- [ ] Position estimates smoothly converge to GPS
- [ ] Velocity matches GPS velocity

### 4.2 Instrumentation for Validation
Add logging to track filter health:

```rust
// Log covariance trace (sum of diagonal)
let cov_trace = (0..15).map(|i| filter.covariance.data[(i, i)]).sum::<f32>();

// Log innovation (measurement residual)
let innovation_norm = (pos_obs.position_ned_m - filter.state.pos).norm();

// Log measurement acceptance rate
measurement_stats.position_accepted += result.accepted as u32;
measurement_stats.total_position_measurements += 1;
```

### 4.3 Comparison Metrics
Compare ESKF output to truth:

**Pre-Flight (Known static):**
- Position error: should be < 10m with GPS
- Velocity error: should be < 1 m/s with GPS
- Attitude error: should be < 5°

**Flight Data Analysis:**
- Plot position, velocity, attitude vs time
- Check convergence speed after GPS initialization
- Look for filter divergence or oscillation
- Verify gravity-aligned Z-velocity matches barometer altitude rate

---

## Phase 5: Tuning & Refinement (Week 3+)

### 5.1 Noise Parameter Tuning
If filter diverges or is too sluggish:

```rust
// Start with conservative values
- Increase gyro_white_noise if attitude drifts
- Increase accel_white_noise if position/velocity diverges  
- Decrease gate thresholds if rejecting valid measurements
- Increase initial_uncertainty if filter doesn't trust measurements enough
```

### 5.2 GPS Coordinate System
Most critical piece - must convert GPS correctly:

```rust
// Example for local NED frame
const REF_LAT: f64 = 41.5;  // Cleveland latitude
const REF_LON: f64 = -81.7;  // Cleveland longitude  
const REF_ALT: f64 = 200.0;  // Reference altitude (m)

fn gps_to_ned(gps_fix: &GpsFix, ref_lat: f64, ref_lon: f64, ref_alt: f64) -> Vec3 {
    // Convert to ECEF
    let ecef_gps = lla_to_ecef(gps_fix.lat, gps_fix.lon, gps_fix.alt);
    let ecef_ref = lla_to_ecef(ref_lat, ref_lon, ref_alt);
    
    // Rotate to NED at reference point
    ecef_to_ned(&ecef_gps, &ecef_ref, ref_lat, ref_lon)
}
```

---

## Implementation Steps (Recommended Order)

1. **Week 1, Day 1:** Set up Phase 1 (config & init)
2. **Week 1, Day 2-3:** Phase 2 (IMU integration)
   - Ground test: verify IMU propagation works
3. **Week 1, Day 4-5:** Phase 3.1 (barometer)
   - Ground test: verify altitude estimates
4. **Week 2, Day 1-2:** Phase 3.2-3.3 (GPS + magnetometer)
   - Ground test: verify measurement fusion
5. **Week 2, Day 3-5:** Phase 4 (comprehensive testing)
   - Extensive ground validation
6. **Week 3:** Phase 5 (tuning)
   - Fly with instrumentation
   - Analyze logs and refine parameters

---

## Success Criteria

### Mathematical Correctness (Primary Goal)
- [ ] Filter covariance remains positive definite throughout flight
- [ ] Measurements accepted/rejected with sensible gate logic
- [ ] State estimates converge to measurements (not diverge from them)
- [ ] Kalman gains decay over time as covariance shrinks
- [ ] No NaN/Inf values in state or covariance

### Hardware Validation
- [ ] Position estimates match GPS within measurement noise
- [ ] Velocity estimates smooth and reasonable
- [ ] Attitude estimate matches actual rocket orientation
- [ ] Filter survives full flight without divergence
- [ ] Post-flight analysis shows mathematically consistent behavior

---

## Risk Mitigation

**If filter diverges:**
1. Check coordinate frame conversions (most common issue)
2. Verify sensor calibration/bias values
3. Increase initial covariance uncertainty
4. Reduce gate thresholds to accept measurements sooner
5. Check measurement timestamps are monotonically increasing

**If measurements are rejected:**
1. Verify variance/noise parameters are realistic
2. Check gate thresholds (default 3.0 σ is often too tight)
3. Ensure measurements have valid timestamps
4. Verify coordinate transformations are correct

**If attitude flips:**
1. Check quaternion sign consistency (q and -q are same rotation)
2. Verify magnetometer reference field is correct for location
3. Check initial attitude initialization

