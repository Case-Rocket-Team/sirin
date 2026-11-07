#![allow(non_camel_case_types)]
#![cfg_attr(not(test), no_std)]

pub const EKF_STATE_DIM: usize = 18;
pub const EKF_CONTROL_DIM: usize = 6;


#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct vec3f{
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct quatf{
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ekf_settings{
    pub accel_var_f2ps4: f32,
    pub accel_bias_var_f2ps4: f32,

    pub gyro_var_rad2ps2: f32,
    pub gyro_bias_var_rad2ps2: f32,

    pub baro_var_ft2: f32,

    pub magn_covar_gauss2: [f32; 3*3],
    pub magn_bias_var_gauss2: f32,

    pub gnss_covar_ft2: [f32; 3*3],
}

#[repr(C)]
#[derive(Clone)]
pub struct extended_kalman_filter{
    pub nominal_state: ekf_nominal_state,

    pub settings: ekf_settings,

    pub world_magn_north_gauss: vec3f,
    
    // Timestamp of the current state estimate
    pub state_time_us: u32,

    // Most recent control input, cached so that we can predict to the exact
    // timestamp needed during a given update/prediction step
    pub control_input: ekf_control_input,

    // Covariance for the *error* state
    pub state_covar: [f32; EKF_STATE_DIM*EKF_STATE_DIM],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ekf_nominal_state{
    // Stored such that left multiplication by attitude goes from body frame
    // to world frame
    pub attitude: quatf,

    // Velocity is in the world's frame of reference
    pub vel_fps: vec3f,

    // Position is in the world's frame of reference
    pub pos_ft: vec3f,

    pub gyro_bias_radps: vec3f, // radians / sec
    pub accel_bias_fps2: vec3f,
    pub magn_bias_gauss: vec3f,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ekf_control_input_vars{
    pub accel_fps2: vec3f,
    pub gyro_radps: vec3f, 
}

#[repr(C)]
#[derive(Clone, Copy)]
pub union ekf_control_input{
    pub vars: ekf_control_input_vars,
    pub v: [f32; EKF_CONTROL_DIM],
}

unsafe extern "C" {
    pub fn ekf_init(ekf: *mut extended_kalman_filter, settings: *const ekf_settings, local_down: vec3f, local_magn_gauss: vec3f, timestamp_us: u32);
    pub fn ekf_inject_imu(ekf: *mut extended_kalman_filter, accel_fps2: vec3f, gyro_radps: vec3f, timestamp_us: u32);
    pub fn ekf_inject_baro(ekf: *mut extended_kalman_filter, altitude_ft: f32, timestamp_us: u32);
    pub fn ekf_inject_magn(ekf: *mut extended_kalman_filter, magn_north_gauss: vec3f, timestamp_us: u32);
    pub fn ekf_inject_gnss(ekf: *mut extended_kalman_filter, pos_ft: vec3f, timestamp_us: u32);
}


const FT_TO_M: f32 = 0.3048;
const M_TO_FT: f32 = 1.0 / FT_TO_M;

fn m_to_ft(v: vec3f) -> vec3f {
    vec3f { x: v.x * M_TO_FT, y: v.y * M_TO_FT, z: v.z * M_TO_FT }
}

fn ft_to_m(v: vec3f) -> vec3f {
    vec3f { x: v.x * FT_TO_M, y: v.y * FT_TO_M, z: v.z * FT_TO_M }
}

pub struct Ekf {
    raw: extended_kalman_filter,
}

impl Ekf {
    pub fn new(settings: &ekf_settings, local_down: vec3f, local_magn_gauss: vec3f, timestamp_ms: u32) -> Self {
        let timestamp_us = timestamp_ms.wrapping_mul(1000);
        let mut raw = core::mem::MaybeUninit::<extended_kalman_filter>::uninit();
        unsafe {
            ekf_init(raw.as_mut_ptr(), settings, local_down, local_magn_gauss, timestamp_us);
            Self { raw: raw.assume_init() }
        }
    }
    pub fn inject_imu(&mut self, accel_mps2: vec3f, gyro_radps: vec3f, timestamp_ms: u32){
        let timestamp_us = timestamp_ms.wrapping_mul(1000);
        let accel_fps2 = m_to_ft(accel_mps2);
        unsafe { ekf_inject_imu(&mut self.raw, accel_fps2, gyro_radps, timestamp_us) };
    }
    pub fn inject_baro(&mut self, altitude_m: f32, timestamp_ms: u32){
        let timestamp_us = timestamp_ms.wrapping_mul(1000);
        let altitude_ft = altitude_m*M_TO_FT;
        unsafe { ekf_inject_baro(&mut self.raw, altitude_ft, timestamp_us) };
    }
    pub fn inject_magn(&mut self, magn_north_gauss: vec3f, timestamp_ms: u32){
        let timestamp_us = timestamp_ms.wrapping_mul(1000);
        unsafe { ekf_inject_magn(&mut self.raw, magn_north_gauss, timestamp_us) };
    }
    pub fn inject_gnss(&mut self, pos_m: vec3f, timestamp_ms: u32){
        let timestamp_us = timestamp_ms.wrapping_mul(1000);
        let pos_ft = m_to_ft(pos_m);
        unsafe { ekf_inject_gnss(&mut self.raw, pos_ft, timestamp_us) };
    }
    pub fn position_m(&self) -> vec3f {
        return ft_to_m(self.raw.nominal_state.pos_ft);
    }
    pub fn velocity_mps(&self) -> vec3f {
        return ft_to_m(self.raw.nominal_state.vel_fps);
    }
}