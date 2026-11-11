#![allow(non_camel_case_types)]
#![cfg_attr(not(test), no_std)]

const EKF_STATE_DIM: usize = 18;
const EKF_CONTROL_DIM: usize = 6;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct vec3f {
    x: f32,
    y: f32,
    z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct quatf {
    w: f32,
    x: f32,
    y: f32,
    z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ekf_settings {
    pub accel_var_f2ps4: f32,
    pub accel_bias_var_f2ps4: f32,

    pub gyro_var_rad2ps2: f32,
    pub gyro_bias_var_rad2ps2: f32,

    pub baro_var_ft2: f32,

    pub magn_covar_gauss2: [f32; 3 * 3],
    pub magn_bias_var_gauss2: f32,

    pub gnss_covar_ft2: [f32; 3 * 3],
}

#[repr(C)]
#[derive(Clone)]
struct extended_kalman_filter {
    nominal_state: ekf_nominal_state,

    settings: ekf_settings,

    world_magn_north_gauss: vec3f,

    // Timestamp of the current state estimate
    state_time_us: u32,

    // Most recent control input, cached so that we can predict to the exact
    // timestamp needed during a given update/prediction step
    control_input: ekf_control_input,

    // Covariance for the *error* state
    state_covar: [f32; EKF_STATE_DIM * EKF_STATE_DIM],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct ekf_nominal_state {
    // Stored such that left multiplication by attitude goes from body frame
    // to world frame
    attitude: quatf,

    // Velocity is in the world's frame of reference
    vel_fps: vec3f,

    // Position is in the world's frame of reference
    pos_ft: vec3f,

    gyro_bias_radps: vec3f, // radians / sec
    accel_bias_fps2: vec3f,
    magn_bias_gauss: vec3f,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
struct ekf_control_input_vars {
    accel_fps2: vec3f,
    gyro_radps: vec3f,
}

#[repr(C)]
#[derive(Clone, Copy)]
union ekf_control_input {
    vars: ekf_control_input_vars,
    v: [f32; EKF_CONTROL_DIM],
}

unsafe extern "C" {
    fn ekf_init(ekf: *mut extended_kalman_filter, settings: *const ekf_settings, local_down: vec3f, local_magn_gauss: vec3f, timestamp_us: u32);
    fn ekf_inject_imu(ekf: *mut extended_kalman_filter, accel_fps2: vec3f, gyro_radps: vec3f, timestamp_us: u32);
    fn ekf_inject_baro(ekf: *mut extended_kalman_filter, altitude_ft: f32, timestamp_us: u32);
    fn ekf_inject_magn(ekf: *mut extended_kalman_filter, magn_north_gauss: vec3f, timestamp_us: u32);
    fn ekf_inject_gnss(ekf: *mut extended_kalman_filter, pos_ft: vec3f, timestamp_us: u32);
}



use sirin_shared::{
    packet::Vec3,
    state::{Accel, AngularVel, Pos, Quaternion, Vel},
};
use uunit::{Meters, Quantity};

const FT_TO_M: f32 = 0.3048;
const M_TO_FT: f32 = 1.0 / FT_TO_M;


fn pos_to_vec3f(p: &Pos) -> vec3f {
    vec3f {
        x: p.x.value * M_TO_FT,
        y: p.y.value * M_TO_FT,
        z: p.z.value * M_TO_FT
    }
}

fn accel_to_vec3f(a: &Accel) -> vec3f {
    vec3f {
        x: a.x.value * M_TO_FT,
        y: a.y.value * M_TO_FT,
        z: a.z.value * M_TO_FT
    }
}

fn angular_vel_to_vec3f(w: &AngularVel) -> vec3f {
    vec3f {
        x: w.x_pitch.value,
        y: w.y_roll.value,
        z: w.z_yaw.value
    }
}

fn gauss_to_vec3f(v: &Vec3<f32>) -> vec3f {
    vec3f {
        x: v.x,
        y: v.y,
        z: v.z
    }
}

fn vec3f_to_pos(v: vec3f) -> Pos {
    Pos {
        x: Quantity::new(v.x * FT_TO_M),
        y: Quantity::new(v.y * FT_TO_M),
        z: Quantity::new(v.z * FT_TO_M)
    }
}

fn vec3f_to_vel(v: vec3f) -> Vel {
    Vel {
        x: Quantity::new(v.x * FT_TO_M),
        y: Quantity::new(v.y * FT_TO_M),
        z: Quantity::new(v.z * FT_TO_M)
    }
}

fn quatf_to_quaternion(q: quatf) -> Quaternion<f32> {
    Quaternion::new(q.w, q.x, q.y, q.z)
}

pub struct Ekf {
    raw: extended_kalman_filter
}

impl Ekf {
    pub fn new(settings: &ekf_settings, local_down: &Accel, local_magn_gauss: &Vec3<f32>, timestamp_us: u32) -> Self {
        let mut raw = core::mem::MaybeUninit::<extended_kalman_filter>::uninit();
        let local_down_vec3f = accel_to_vec3f(local_down);
        let local_magn_gauss_vec3f = gauss_to_vec3f(local_magn_gauss);
        unsafe {
            ekf_init(raw.as_mut_ptr(), settings, local_down_vec3f, local_magn_gauss_vec3f, timestamp_us);
            Self {
                raw: raw.assume_init()
            }
        }
    }
    pub fn inject_imu(&mut self, accel: &Accel, gyro: &AngularVel, timestamp_us: u32) {
        let accel_fps2 = accel_to_vec3f(accel);
        let gyro_radps = angular_vel_to_vec3f(gyro);
        unsafe {
            ekf_inject_imu(&mut self.raw, accel_fps2, gyro_radps, timestamp_us)
        };
    }
    pub fn inject_baro(&mut self, altitude: Meters<f32>, timestamp_us: u32) {
        let altitude_ft = altitude.value * M_TO_FT;
        unsafe {
            ekf_inject_baro(&mut self.raw, altitude_ft, timestamp_us)
        };
    }
    pub fn inject_magn(&mut self, magn_gauss: &Vec3<f32>, timestamp_us: u32) {
        let magn_north_gauss = gauss_to_vec3f(magn_gauss);
        unsafe {
            ekf_inject_magn(&mut self.raw, magn_north_gauss, timestamp_us)
        };
    }
    pub fn inject_gnss(&mut self, pos: &Pos, timestamp_us: u32) {
        let pos_ft = pos_to_vec3f(pos);
        unsafe {
            ekf_inject_gnss(&mut self.raw, pos_ft, timestamp_us)
        };
    }
    pub fn position(&self) -> Pos {
        vec3f_to_pos(self.raw.nominal_state.pos_ft)
    }
    pub fn velocity(&self) -> Vel {
        vec3f_to_vel(self.raw.nominal_state.vel_fps)
    }
    pub fn attitude(&self) -> Quaternion<f32> {
        quatf_to_quaternion(self.raw.nominal_state.attitude)
    }
}