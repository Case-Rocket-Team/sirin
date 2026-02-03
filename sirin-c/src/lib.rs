#![no_std]
#![doc = include_str!("../README.md")]
#![allow(unused_imports)]
#![allow(unused_variables)]
use core::ffi::CStr;

use defmt::Debug2Format;
use sirin_shared::state::{Accel, AngularVel, CovarianceMatrixP, ErrorState, NominalState};
use uunit::{Hectopascals, Meters, MetersPerSecond2, Seconds};

#[no_mangle]
pub extern "C" fn sirin_log(str: *const core::ffi::c_char) {
    let c_str = unsafe {
        CStr::from_ptr(str)
    };
    // defmt::info!("sirin-c: {}", Debug2Format(&c_str))
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
        angular_vel_measurement: *const f32
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