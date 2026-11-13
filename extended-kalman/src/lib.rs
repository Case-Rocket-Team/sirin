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
pub struct Ekf_settings {
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

    settings: Ekf_settings,

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
    fn ekf_init(ekf: *mut extended_kalman_filter, settings: *const Ekf_settings, local_down: vec3f, local_magn_gauss: vec3f, timestamp_us: u32);
    fn ekf_inject_imu(ekf: *mut extended_kalman_filter, accel_fps2: vec3f, gyro_radps: vec3f, timestamp_us: u32);
    fn ekf_inject_baro(ekf: *mut extended_kalman_filter, altitude_ft: f32, timestamp_us: u32);
    fn ekf_inject_magn(ekf: *mut extended_kalman_filter, magn_north_gauss: vec3f, timestamp_us: u32);
    fn ekf_inject_gnss(ekf: *mut extended_kalman_filter, pos_ft: vec3f, timestamp_us: u32);
}



use core::f32::consts::PI;

use sirin_shared::{
    packet::{ImuData, MagnetometerData, Vec3},
    state::{Pos, Quaternion, Vel, Accel, AngularVel},
};
use uunit::{MicroGs, Meters, Quantity};

const FT_TO_M: f32 = 0.3048;
const M_TO_FT: f32 = 1.0 / FT_TO_M;
const G_FTPS2: f32 = 9.80665 * M_TO_FT;
const MAG_LSB_PER_GAUSS: f32 = 6842.0;

const ACCEL_OFFSET_G: [f32; 3] = [0.00035467744, -0.000054717064, 0.008441657];
const ACCEL_SCALE: [f32; 3] = [1.0079681, 1.0090547, 1.0095128];

fn calibrate_g(reading_g: f32, axis: usize) -> f32 {
    (reading_g - ACCEL_OFFSET_G[axis]) * ACCEL_SCALE[axis]
}


fn pos_to_vec3f(p: &Pos) -> vec3f {
    vec3f {
        x: p.x.value * M_TO_FT,
        y: p.y.value * M_TO_FT,
        z: p.z.value * M_TO_FT
    }
}

fn accel_to_vec3f(a: &Vec3<MicroGs<i32>>) -> vec3f {
    vec3f {
        x: calibrate_g(a.x.value as f32 * 1e-6, 0) * G_FTPS2,
        y: calibrate_g(a.y.value as f32 * 1e-6, 1) * G_FTPS2,
        z: calibrate_g(a.z.value as f32 * 1e-6, 2) * G_FTPS2
    }
}

fn angular_vel_to_vec3f(w: &ImuData) -> Option<vec3f> {
    let w = w.angular_vel.as_ref().ok()?;
    Some(vec3f {
        x: w.x.value as f32 * 1e-6 * (PI / 180.0),
        y: w.y.value as f32 * 1e-6 * (PI / 180.0),
        z: w.z.value as f32 * 1e-6 * (PI / 180.0)
    })
}


fn gauss_to_vec3f(m: &MagnetometerData) -> Option<vec3f> {
    let m = m.mag.as_ref().ok()?;
    Some(vec3f {
        x: m.x as f32 / MAG_LSB_PER_GAUSS,
        y: m.y as f32 / MAG_LSB_PER_GAUSS,
        z: m.z as f32 / MAG_LSB_PER_GAUSS
    })
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
    pub fn new(settings: &Ekf_settings, imu: &Accel, gyro_bias: &AngularVel, mag: &MagnetometerData, timestamp_us: u32) -> Option<Self> {
        let mut raw = core::mem::MaybeUninit::<extended_kalman_filter>::uninit();
        const G_MPS2: f32 = 9.80665;
        let accel = vec3f {
            x: calibrate_g(imu.x.value / G_MPS2, 0) * G_FTPS2,
            y: calibrate_g(imu.y.value / G_MPS2, 1) * G_FTPS2,
            z: calibrate_g(imu.z.value / G_MPS2, 2) * G_FTPS2,
        };
        let local_down_vec3f = vec3f { x: -accel.x, y: -accel.y, z: -accel.z };
        let local_magn_gauss_vec3f = gauss_to_vec3f(mag)?;
        unsafe {
            ekf_init(raw.as_mut_ptr(), settings, local_down_vec3f, local_magn_gauss_vec3f, timestamp_us);
            let mut raw = raw.assume_init();
            raw.nominal_state.gyro_bias_radps = vec3f {
                x: gyro_bias.x_pitch.value,
                y: gyro_bias.y_roll.value,
                z: gyro_bias.z_yaw.value,
            };
            Some(Self { raw })
        }
    }
    pub fn inject_imu(&mut self, imu: &ImuData, timestamp_us: u32) {
        let (Ok(accel), Some(gyro_radps)) = (&imu.accel, angular_vel_to_vec3f(imu)) else {
            return;
        };
        let accel_fps2 = accel_to_vec3f(accel);
        unsafe {
            ekf_inject_imu(&mut self.raw, accel_fps2, gyro_radps, timestamp_us)
        };
    }
    pub fn inject_baro(&mut self, altitude: Meters<f64>, timestamp_us: u32) {
        let altitude_ft = altitude.value as f32 * M_TO_FT;
        unsafe {
            ekf_inject_baro(&mut self.raw, altitude_ft, timestamp_us)
        };
    }
    pub fn inject_magn(&mut self, mag: &MagnetometerData, timestamp_us: u32) {
        let Some(magn_north_gauss) = gauss_to_vec3f(mag) else {
            return;
        };
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