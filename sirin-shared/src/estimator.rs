/// Monotonic physical sample time and driver delivery time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SampleTime {
    pub timestamp_us: u64,
    pub arrival_timestamp_us: u64,
    pub sequence: u32,
}

#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub time: SampleTime,
    pub accel_mps2_b: [f32; 3],
    pub gyro_radps_b: [f32; 3],
    pub temperature_c: Option<f32>,
    pub accel_saturation_mask: u8,
    pub gyro_saturation_mask: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct GpsSample {
    pub time: SampleTime,
    pub gps_time_of_week_ms: u32,
    pub latitude_rad: f64,
    pub longitude_rad: f64,
    pub ellipsoid_height_m: f64,
    pub msl_height_m: Option<f64>,
    pub ecef_position_m: [f64; 3],
    pub velocity_ned_mps: Option<[f32; 3]>,
    pub position_variance_m2: Option<[f32; 3]>,
    pub velocity_variance_m2ps2: Option<[f32; 3]>,
    pub fix_type: crate::packet::GpsFixType,
    pub satellites: u8,
    pub valid: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct BarometerSample {
    pub time: SampleTime,
    pub pressure_pa: f32,
    pub temperature_c: Option<f32>,
    pub variance_pa2: Option<f32>,
    pub valid: bool,
}

#[derive(Debug, Clone, Copy)]
pub struct MagnetometerSample {
    pub time: SampleTime,
    pub field_ut_b: [f32; 3],
    pub saturation_mask: u8,
    pub valid: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct NavigationValidity {
    pub attitude: bool,
    pub heading: bool,
    pub horizontal_velocity: bool,
    pub vertical_velocity: bool,
    pub local_horizontal_position: bool,
    pub local_vertical_position: bool,
    pub global_position: bool,
    pub covariance: bool,
    pub time_sync: bool,
}

/// Explicitly framed estimator output for logging, telemetry adapters, and
/// flight logic. Quaternion order is Hamilton scalar-first [w, x, y, z].
#[derive(Debug, Clone, Copy)]
pub struct NavigationSolution {
    pub timestamp_us: u64,
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_nb_wxyz: [f32; 4],
    pub accel_bias_mps2_b: [f32; 3],
    pub gyro_bias_radps_b: [f32; 3],
    pub validity: NavigationValidity,
}
