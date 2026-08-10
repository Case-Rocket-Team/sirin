use crate::state::Vec3;

#[derive(Debug, Clone)]
pub struct ImuNoise {
    pub accel_noise_density_mps2_sqrt_hz: Vec3,
    pub gyro_noise_density_radps_sqrt_hz: Vec3,
    pub accel_bias_random_walk_mps3_sqrt_hz: Vec3,
    pub gyro_bias_random_walk_radps2_sqrt_hz: Vec3,
}

impl Default for ImuNoise {
    fn default() -> Self {
        Self {
            // Conservative placeholders. Replace with measured Allan results.
            accel_noise_density_mps2_sqrt_hz: Vec3::repeat(0.02),
            gyro_noise_density_radps_sqrt_hz: Vec3::repeat(0.002),
            accel_bias_random_walk_mps3_sqrt_hz: Vec3::repeat(0.0005),
            gyro_bias_random_walk_radps2_sqrt_hz: Vec3::repeat(0.00005),
        }
    }
}

#[derive(Debug, Clone)]
pub struct InitialUncertainty {
    pub position_std_m: Vec3,
    pub velocity_std_mps: Vec3,
    pub attitude_std_rad: Vec3,
    pub accel_bias_std_mps2: Vec3,
    pub gyro_bias_std_radps: Vec3,
}

impl Default for InitialUncertainty {
    fn default() -> Self {
        Self {
            position_std_m: Vec3::new(5.0, 5.0, 10.0),
            velocity_std_mps: Vec3::repeat(2.0),
            attitude_std_rad: Vec3::new(
                10.0_f32.to_radians(),
                10.0_f32.to_radians(),
                45.0_f32.to_radians(),
            ),
            accel_bias_std_mps2: Vec3::repeat(0.5),
            gyro_bias_std_radps: Vec3::repeat(0.05),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EskfConfig {
    pub gravity_ned_mps2: Vec3,
    pub min_imu_dt_s: f32,
    pub max_imu_dt_s: f32,
    pub max_measurement_age_us: u64,
    pub future_measurement_tolerance_us: u64,
    pub imu_noise: ImuNoise,
    pub initial_uncertainty: InitialUncertainty,
    pub gps_position_variance_m2: Vec3,
    pub gps_velocity_variance_m2ps2: Vec3,
    pub gps_position_gate: f32,
    pub gps_velocity_gate: f32,
    pub barometer_gate: f32,
    pub magnetometer_gate: f32,
    pub magnetic_field_ned_ut: Vec3,
}

impl Default for EskfConfig {
    fn default() -> Self {
        Self {
            gravity_ned_mps2: Vec3::new(0.0, 0.0, 9.80665),
            min_imu_dt_s: 1.0e-6,
            max_imu_dt_s: 0.05,
            max_measurement_age_us: 250_000,
            future_measurement_tolerance_us: 1_000,
            imu_noise: ImuNoise::default(),
            initial_uncertainty: InitialUncertainty::default(),
            gps_position_variance_m2: Vec3::new(2.25, 2.25, 6.25),
            gps_velocity_variance_m2ps2: Vec3::repeat(0.25),
            // Approximate 99.7% chi-squared thresholds for 3 and 1 DOF.
            gps_position_gate: 14.16,
            gps_velocity_gate: 14.16,
            barometer_gate: 9.0,
            magnetometer_gate: 14.16,
            // Placeholder near Cleveland. Replace with a launch-site model.
            magnetic_field_ned_ut: Vec3::new(19.0, -2.7, 48.0),
        }
    }
}
