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

    pub world_magn_north_guass: vec3f,
    pub magn_covar_gauss2: [f32; 3*3],
    pub magn_bias_var_gauss2: f32,

    pub gnss_covar_ft2: [f32; 3*3],
}

#[repr(C)]
#[derive(Clone)]
pub struct extended_kalman_filter{
    pub nominal_state: ekf_nominal_state,

    pub settings: ekf_settings,
    
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