#![no_std]
#![doc = include_str!("../README.md")]

use sirin_shared::state::{Accel, AngularVel, NominalState};
use uunit::Seconds;

extern "C" {
    pub fn update_nominal(
        state: &mut NominalState,
        dt: Seconds<f32>,
        accel_measurement: *const f32,
        angular_vel_measurement: *const f32
    );
}