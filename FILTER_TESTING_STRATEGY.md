# Filter Testing Strategy - Validating State Estimation Math

## High-Level Goal
Replace the old filter in nosecone-ebay with the new ESKF to **validate that the state estimation mathematics is correct** on actual hardware.

---

## What We're Testing

The new ESKF implements a mathematically rigorous error-state Kalman filter with:
1. **IMU propagation**: F matrix for pose and velocity integration
2. **GPS fusion**: H matrix maps position/velocity state to measurements
3. **Barometer fusion**: H matrix maps altitude state to pressure measurement
4. **Magnetometer fusion**: H matrix maps attitude state to magnetic field
5. **Fixed-lag smoothing**: Handles delayed measurements (GPS/baro)

**Success means:** State estimates converge toward true values in a mathematically consistent way.

---

## Testing Phases

### Phase 1: Simulation Validation (Done ✓)
- [ ] Run `cargo run --bin eskf-sim`
- [ ] Verify: Position error < 1m, velocity error < 0.5 m/s
- **This proves:** Math is correct in ideal conditions

### Phase 2: Benchtop Ground Test (1 day)
Stationary on desk with power but no flight:
- [ ] Filter initializes without panicking
- [ ] IMU propagation every 100ms succeeds
- [ ] Attitude estimate is stable when stationary
- [ ] Position/velocity stay near zero
- [ ] Covariance remains positive definite (no divergence)

**Test:** 30 minutes stationary, monitoring log output
```bash
# Watch the filter state
tail -f /tmp/filter_log.txt | grep "Filter covariance\|Position NED\|Velocity NED"
```

### Phase 3: Handheld Motion Test (1-2 hours)
Hold the rocket and move it around:
- [ ] Roll/pitch/yaw changes show in attitude estimate
- [ ] Velocity integrates from acceleration
- [ ] Velocity goes to zero after motion stops
- [ ] No attitude "flips" or sign inversions

**Test Motion:**
1. Tilt forward/back (pitch)
2. Tilt left/right (roll)
3. Rotate in plane (yaw)
4. Accelerate in one direction, coast to stop

### Phase 4: Outdoor Stationary Test (1-2 hours)
Let GPS lock and monitor fusion:
- [ ] GPS position measurements are accepted
- [ ] Position estimates move toward GPS values
- [ ] Velocity estimates converge to zero (stationary)
- [ ] Magnetometer accepts measurements
- [ ] No jumps or divergence when measurements arrive

**Success criteria:**
- GPS position error < 5m after 2 minutes
- No covariance divergence (NaN/Inf)
- All measurement acceptance reasonable

### Phase 5: Flight Test (Multiple flights)
Real rocket flight with complete data logging:

**Before Each Flight:**
- [ ] Fresh initialization on launch pad
- [ ] Check attitude estimate is level
- [ ] Verify all sensors responding
- [ ] Confirm GPS not locked (to test altitude-only mode)

**Data to Log Every 100ms:**
```rust
// In main loop:
if i % 10 == 0 {  // Every 1 second
    log!(
        "time={} pos=[{},{},{}] vel=[{},{},{}] att=[{},{},{},{}] cov_trace={}",
        current_time_us,
        state.nominal.pos.x, state.nominal.pos.y, state.nominal.pos.z,
        state.nominal.vel.x, state.nominal.vel.y, state.nominal.vel.z,
        state.nominal.rot_quaternion.w, state.nominal.rot_quaternion.i, 
        state.nominal.rot_quaternion.j, state.nominal.rot_quaternion.k,
        filter_covariance_trace(),
    );
}
```

**Post-Flight Analysis:**
1. Plot position, velocity, attitude vs time
2. Check for divergence (covariance growing unbounded)
3. Verify attitude looks reasonable
4. Compare altitude estimate to barometer
5. Look for discontinuities or jumps (measurement rejection)

---

## Mathematical Validation Checklist

### Propagation (F matrix) ✓
- [x] Simulator shows correct integration
- [ ] Hardware: velocity integrates from acceleration
- [ ] Hardware: position integrates from velocity
- [ ] Hardware: attitude rotates with gyro input

### GPS Measurement (H matrix position/velocity)
- [ ] Simulator: accepts delayed measurements
- [ ] Hardware: accepts GPS position when available
- [ ] Hardware: position estimates converge to GPS
- [ ] Hardware: velocity estimates converge to GPS velocity

### Barometer Measurement (H matrix altitude)
- [ ] Simulator: altitude correct (tested with height_up_m)
- [ ] Hardware: barometer measurements accepted
- [ ] Hardware: altitude estimate matches pressure altitude
- [ ] Hardware: vertical velocity matches altitude rate

### Magnetometer Measurement (H matrix yaw)
- [ ] Simulator: accepts magnetic field
- [ ] Hardware: magnetometer measurements accepted
- [ ] Hardware: heading estimates stabilize
- [ ] Hardware: no yaw oscillation or flipping

### Covariance Matrix
- [ ] Simulator: stays positive definite throughout
- [ ] Hardware: no NaN/Inf values
- [ ] Hardware: trace decreases with measurements (uncertainty shrinks)
- [ ] Hardware: trace grows between measurements (process noise)

### Kalman Gain
- [ ] Early in flight: large gains (trust measurements)
- [ ] After convergence: small gains (trust propagation)
- [ ] Gates reject outliers sensibly

---

## Data Analysis Template

After each flight, run this analysis:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load flight log
df = pd.read_csv('flight_log.csv', parse_dates=['time'])

fig, axes = plt.subplots(3, 2, figsize=(14, 10))

# Position (NED)
axes[0,0].plot(df['time'], df['pos_x'], label='X (North)')
axes[0,0].plot(df['time'], df['pos_y'], label='Y (East)')
axes[0,0].plot(df['time'], df['pos_z'], label='Z (Down)')
axes[0,0].set_ylabel('Position (m)')
axes[0,0].legend()
axes[0,0].grid(True)

# Velocity (NED)
axes[0,1].plot(df['time'], df['vel_x'], label='VX')
axes[0,1].plot(df['time'], df['vel_y'], label='VY')
axes[0,1].plot(df['time'], df['vel_z'], label='VZ')
axes[0,1].set_ylabel('Velocity (m/s)')
axes[0,1].legend()
axes[0,1].grid(True)

# Covariance trace (should shrink then grow)
axes[1,0].semilogy(df['time'], df['cov_trace'])
axes[1,0].set_ylabel('Covariance Trace (log scale)')
axes[1,0].grid(True)

# Attitude (should be smooth, bounded)
axes[1,1].plot(df['time'], df['quat_w'], label='w')
axes[1,1].plot(df['time'], df['quat_x'], label='x')
axes[1,1].plot(df['time'], df['quat_y'], label='y')
axes[1,1].plot(df['time'], df['quat_z'], label='z')
axes[1,1].set_ylabel('Quaternion Component')
axes[1,1].legend()
axes[1,1].grid(True)

# Check for divergence
axes[2,0].plot(df['time'], df['cov_trace'].diff())
axes[2,0].set_ylabel('Covariance Rate of Change')
axes[2,0].axhline(y=0, color='r', linestyle='--')
axes[2,0].grid(True)

# Altitude (should match barometer)
if 'baro_altitude' in df.columns:
    axes[2,1].plot(df['time'], -df['pos_z'], label='Filter -Z')
    axes[2,1].plot(df['time'], df['baro_altitude'], label='Barometer')
    axes[2,1].set_ylabel('Altitude (m)')
    axes[2,1].legend()
    axes[2,1].grid(True)

plt.tight_layout()
plt.savefig('filter_analysis.png', dpi=150)
plt.show()

# Print diagnostics
print(f"Max position change: {df[['pos_x','pos_y','pos_z']].std().max():.1f} m")
print(f"Max velocity: {df[['vel_x','vel_y','vel_z']].abs().max().max():.1f} m/s")
print(f"Final covariance: {df['cov_trace'].iloc[-1]:.1e}")
print(f"Any NaN values: {df.isna().any().any()}")
```

---

## Red Flags (Stop and Debug)

**STOP if you see:**
1. **NaN or Inf in covariance** → Math error or numerical overflow
2. **Covariance monotonically increases** → Measurements being rejected or process noise too large
3. **Attitude flips 180°** → Quaternion sign ambiguity not handled
4. **Position diverges to ±∞** → Sensor calibration or coordinate frame wrong
5. **Sudden jumps in state** → Measurement gate too loose or timestamp issue
6. **All measurements rejected** → Gate thresholds too tight or noise parameters wrong

---

## Success Criteria by Phase

### Phase 2 (Benchtop)
```
✓ No panics or errors
✓ Covariance is finite
✓ State is bounded
✓ IMU propagation succeeds
```

### Phase 3 (Handheld)
```
✓ Attitude changes with motion
✓ Velocity integrates correctly
✓ No jumps or flips
✓ Covariance remains positive definite
```

### Phase 4 (GPS Ground)
```
✓ GPS position accepted
✓ Position converges to GPS
✓ Velocity converges to zero
✓ Covariance shrinks
✓ No divergence
```

### Phase 5 (Flight)
```
✓ Completes without divergence
✓ Position/velocity stay bounded
✓ Attitude reasonable (~1-2 rad from true)
✓ Altitude estimate sensible
✓ Post-flight analysis shows consistent behavior
```

---

## Questions to Answer After Each Flight

1. **Did the state estimate diverge?**
   - If yes: Check covariance trace, look for NaN/Inf
   
2. **Did measurements get accepted?**
   - Check GPS/barometer/mag fusion results in log
   - If all rejected: tune gate thresholds
   
3. **Did the attitude estimate match reality?**
   - Can you estimate true attitude from rocket orientation?
   - Does filter attitude match?
   
4. **Did the position make physical sense?**
   - Rocket launched vertically, so mostly Z (down) motion
   - Some X/Y motion from wind, but should be small
   
5. **Did the velocity match altitude rate?**
   - dZ/dt should match -velocity_z (down is positive)
   - Check against barometer altitude changes

---

## Success Threshold

**The filter is mathematically correct on hardware when:**

1. Covariance remains finite and positive definite (no divergence)
2. State estimates are bounded and physically reasonable
3. Measurements are accepted/rejected appropriately
4. Estimates converge toward measurements in sensible timeframes
5. Post-flight analysis shows no mathematical inconsistencies

**This proves:** The Kalman filter implementation correctly represents and solves the state estimation problem.

