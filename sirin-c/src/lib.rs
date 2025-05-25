#![no_std]
#![doc = include_str!("../README.md")]

use sirin_shared::state::{Accel, AngularVel, ErrorState, NominalState};
use uunit::{Hectopascals, Meters, MetersPerSecond2, Seconds};

extern "C" {
    pub fn update_with_imu(
        nominal: &mut NominalState,
        error: &mut ErrorState,
        dt: Seconds<f32>,
        accel_measurement: *const f32,
        angular_vel_measurement: *const f32
    );

    /// Calculate altitude exactly from pressure using libm
    pub fn pressure_altitude(
        pressure: Hectopascals<f64>
    ) -> Meters<f64>;

    pub fn gravity_at_altitude(
        pressure: Hectopascals<f32>
    ) -> MetersPerSecond2<f32>;
}