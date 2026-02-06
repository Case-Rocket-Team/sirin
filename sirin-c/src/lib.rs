#![no_std]
#![doc = include_str!("../README.md")]

use core::{ffi::CStr, ptr::slice_from_raw_parts};

use defmt::Debug2Format;
use sirin_shared::state::{Accel, AngularVel, CovarianceMatrixP, ErrorState, NominalState};
use uunit::{Hectopascals, Meters, MetersPerSecond2, Seconds};

#[no_mangle]
pub extern "C" fn sirin_log(lvl: u32, str: *const core::ffi::c_char) {
    let c_str = unsafe {
        CStr::from_ptr(str)
    };

    match lvl {
        1 => defmt::debug!("sirin-c: {}", Debug2Format(&c_str)),
        2 => defmt::info!("sirin-c: {}", Debug2Format(&c_str)),
        3 => defmt::warn!("sirin-c: {}", Debug2Format(&c_str)),
        _ => defmt::panic!("sirin-c: {}", Debug2Format(&c_str)),
    }
    
}

#[no_mangle]
pub extern "C" fn sirin_log_f32_array(ptr: *const f32, len: usize) {
    let slice = unsafe {
        &*slice_from_raw_parts(ptr, len)
    };

    defmt::info!("sirin-c log f32 array: {}", slice);
}

extern "C" {
    pub fn update_with_imu(
        nominal: &mut NominalState,
        error: &mut ErrorState,
        cov: &mut CovarianceMatrixP,
        dt: Seconds<f32>,
        accel_measurement: *const f32,
        angular_vel_measurement: *const f32
    );

    pub fn init_with_imu(
        nominal: &mut NominalState,
        error: &mut ErrorState,
        accel_measurement: *const f32,
        angular_vel_measurement: *const f32,
        magnetometer: *const f32
    );

    pub fn correct_from_gps(
        nominal: &mut NominalState,
        error: &mut ErrorState,
        cov: &mut CovarianceMatrixP,
        gps_position: *const f64,
        gps_velocity: *const f32
    );

    /// Calculate altitude exactly from pressure using libm
    //pub fn pressure_altitude(
    //    pressure: Hectopascals<f64>
    //) -> Meters<f64>;

    pub fn gravity_at_altitude(
        pressure: Hectopascals<f32>
    ) -> MetersPerSecond2<f32>;
}