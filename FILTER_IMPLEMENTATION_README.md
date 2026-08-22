# ESKF Integration for nosecone-ebay - Complete Guide

## Overview

This directory contains a complete plan for replacing the existing filter in `examples/nosecone-ebay` with the new **Error-State Kalman Filter (ESKF)** implementation to properly validate state estimation mathematics on hardware.

**Primary Goal:** Ensure the filter correctly propagates IMU data and fuses GPS/barometer/magnetometer measurements according to Kalman filter theory.

---

## Documents

### 1. **FILTER_REPLACEMENT_PLAN.md** ⭐ START HERE
Complete 5-phase implementation roadmap:
- Phase 1: Configuration & Initialization
- Phase 2: IMU Integration  
- Phase 3: Measurement Fusion (Barometer, GPS, Magnetometer)
- Phase 4: Testing & Validation
- Phase 5: Tuning & Refinement

**Read this to understand the overall strategy and timeline.**

### 2. **FILTER_INTEGRATION_CODE.md**
Concrete code snippets ready to copy-paste:
- Helper functions for initialization and unit conversion
- Exact code to replace line 248 in main.rs
- Phase-by-phase code additions
- Common pitfalls to avoid

**Read this when you're ready to write code.**

### 3. **FILTER_TESTING_STRATEGY.md**
How to validate mathematical correctness:
- Ground testing checklist
- Flight data analysis template
- Red flags that indicate math errors
- Success criteria by phase

**Read this before flying to know what to measure.**

---

## Quick Start

### If you have 2 hours (Quick Assessment):
1. Read **FILTER_REPLACEMENT_PLAN.md** (30 min)
2. Read **FILTER_INTEGRATION_CODE.md** Phase 1-2 sections (45 min)
3. Identify the changes needed in your codebase (45 min)

### If you have 1 week (Implementation):
- **Day 1-2:** Phase 1 (configuration & initialization)
  - Implement helper functions
  - Set up filter config
  - Ground test initialization
  
- **Day 3:** Phase 2 (IMU integration)
  - Add IMU measurement processing
  - Verify IMU propagation works
  - Test benchtop
  
- **Day 4-5:** Phase 3 (measurement fusion)
  - Add barometer fusion
  - Add GPS integration (if GPS available)
  - Ground test with measurements
  
- **Day 6-7:** Phase 4 (validation)
  - Comprehensive testing
  - Data analysis
  - Parameter tuning if needed

### If you have 3 weeks (Complete & Flight):
Follow all 5 phases in **FILTER_REPLACEMENT_PLAN.md** with full validation at each step.

---

## Key Implementation Points

### Critical Files to Modify
```
examples/nosecone-ebay/src/main.rs
├── Line ~17: Add sirin_filter imports
├── Line ~99: Add filter config and initialization  
├── Line ~122: Add timestamp tracking
├── Line ~248: Replace old filter code with new ESKF
└── Add measurement fusion code (barometer, GPS, mag)
```

### Critical Decisions
1. **Magnetic field reference**: Update for your location (Cleveland: 19000, -2700, 48000 μT)
2. **Coordinate frames**: Uses NED (North-East-Down), not ECEF
3. **Unit conversions**: micro-Gs (1e-6) and micro-Deg/s (1e-6)
4. **Gate thresholds**: Start at 3.0 σ, adjust if needed
5. **History lag**: 64 samples at 10Hz ≈ 6.4 seconds (sufficient for most delays)

### Critical Math Validations
- [ ] IMU propagation: velocity integrates from acceleration
- [ ] GPS fusion: position estimates converge to GPS
- [ ] Barometer fusion: altitude matches pressure altitude
- [ ] Attitude estimate: quaternion doesn't flip or oscillate
- [ ] Covariance: stays positive definite, no NaN/Inf

---

## Architecture

```
nosecone-ebay (main.rs)
    │
    ├─→ ESKF (error-state Kalman filter)
    │   ├─→ Propagate with IMU (100 Hz)
    │   └─→ Fuse measurements (variable rate)
    │
    ├─→ FixedLagHistory (smoother for delayed measurements)
    │   ├─→ Buffer past states
    │   └─→ Process GPS/baro with delay
    │
    └─→ Output State
        ├─→ Position (NED, meters)
        ├─→ Velocity (NED, m/s)
        ├─→ Attitude (quaternion, body→NED)
        └─→ Logged to flash
```

---

## What Gets Tested

| Component | Tested By | Success Criterion |
|-----------|-----------|-------------------|
| IMU Propagation (F matrix) | Benchtop motion | Velocity integrates correctly |
| GPS Fusion (H matrix pos/vel) | GPS ground test | Position converges to GPS |
| Barometer Fusion (H matrix alt) | All tests | Altitude matches pressure |
| Magnetometer Fusion (H matrix yaw) | Ground test | Heading stabilizes |
| Covariance Propagation | All tests | No divergence, no NaN |
| State Estimation Convergence | Flight | Estimates bound and reasonable |

---

## Expected Timeline

| Milestone | Duration | Condition |
|-----------|----------|-----------|
| Phase 1 (Init) | 1 day | Filter initializes, attitude reasonable |
| Phase 2 (IMU) | 1 day | IMU propagation works, motion test passes |
| Phase 3 (Sensors) | 2 days | All measurements fused, benchtop stable |
| Phase 4 (Validation) | 2-3 days | Ground test passes all checks |
| Phase 5 (Flight) | 2-3 days | Multiple flights, data analysis clean |
| **Total** | **1-2 weeks** | Mathematically validated filter |

---

## Success Looks Like

### On the Bench
```
[INFO] Filter initialized
[INFO] Initial attitude q=[0.707, 0.0, 0.0, 0.707]
[INFO] IMU propagation: dt=100ms, accel=[0.0, 0.0, -9.8]
[INFO] Filter covariance trace: 1.2e3 (finite, > 0)
[INFO] State: pos=[0, 0, 0] vel=[0, 0, 0] (stable)
```

### During GPS Ground Test
```
[INFO] GPS position received: [100.5, 50.2, 0.0] (NED meters)
[INFO] GPS accepted (innovation < gate)
[INFO] Filter position: [100.2, 50.1, 0.1]
[INFO] Position error: 0.3m (converging)
```

### During Flight
```
[DEBUG] Propagating: vel_z=15.2 m/s (up)
[DEBUG] Barometer accepted: h=245m
[DEBUG] Covariance trace: 500.0 (shrinking, good)
[INFO] Final: pos=[-2.1, 1.3, 250.0] vel=[0.1, -0.2, 0.0]
```

### Post-Flight Analysis
```
✓ No NaN/Inf in covariance
✓ Covariance stayed finite
✓ Position reasonable
✓ Attitude didn't flip
✓ All gates working
✓ Measurements accepted appropriately
```

---

## Troubleshooting Quick Reference

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Filter panics on init | Attitude init failed | Check accel/mag values are non-zero |
| All measurements rejected | Gate thresholds too tight | Reduce from 3.0 to 2.0 σ |
| Covariance diverges to Inf | Measurement noise wrong | Increase variance estimates |
| Position diverges | Coordinate frame wrong | Check NED conversion |
| Attitude flips 180° | Quaternion sign ambiguity | Check sign handling |
| Kalman gain always large | Covariance shrinks too slowly | Increase initial uncertainty |
| Position won't converge | GPS coordinate conversion wrong | Verify lat/lon→NED math |

---

## Next Steps

1. **Read** FILTER_REPLACEMENT_PLAN.md to understand phases 1-5
2. **Study** FILTER_INTEGRATION_CODE.md for exact code changes
3. **Review** FILTER_TESTING_STRATEGY.md for validation approach
4. **Implement** Phase 1 (configuration) on your branch
5. **Ground test** Phase 1 before moving to Phase 2

---

## Contact / Questions

Key things to validate before moving to next phase:
- Filter initializes without errors ✓
- IMU propagation succeeds every 100ms ✓
- State estimates are bounded (not ±∞) ✓
- Covariance is positive definite (no NaN) ✓
- Measurements are accepted reasonably ✓

If any of these fail, debug that phase before continuing.

---

## References

- **ESKF Theory**: Indirect feedback form error-state Kalman filter
- **Simulator**: `examples/eskf-sim` validates the math in ideal conditions
- **Sensor Specs**: Check accelerometer/gyro datasheets for white noise values
- **Magnetic Field**: https://www.ngdc.noaa.gov/geomag/declination
- **NED Coordinates**: North-East-Down frame relative to launch site

---

**Status**: Ready to implement ✅  
**Last Updated**: 2026-08-16  
**Next Review**: After Phase 1 ground testing

