// FOR IREC ROCKET - CHECK TO ENSURE THESE VALUES ARE CODED:
// DO NOT PUSH CODE WITH THESE VALUES SIGNIFICANTLY CHANGED

pub const ACCEL_THRESHOLD_GS_SQUARED: f64 = 10.0 * 10.0; //In Gs squared
pub const ALTITUDE_THRESHOLD_M: f64 = 20.0; //In meters
pub const MAIN_DEPLOYMENT_ALTITUDE_M: f64 = 457.2; //In meters (1500 ft)
pub const FLIGHT_DURATION_S: u64 = 600; //In seconds
pub const APOGEE_ERROR_M: f64 = 1.0; //In meters
pub const APOGEE_TIMEOUT_S: u64 = 25; //In seconds
pub const FLIGHT_TIMEOUT_S: u64 = 1200; //In seconds

pub const STANDBY_BROADCAST_HZ: f64 = 0.5; //In Hz
pub const FLIGHT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const DESCENT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const LANDED_BROADCAST_HZ: f64 = 0.025; //In Hz
pub const LOOP_RATE_HZ: f64 = 10.0; //Main loop rate (100 ms ticker)
pub const USB_BROADCAST_HZ: f64 = 1.0; //In Hz

pub const LANDED_VELOCITY_THRESHOLD_M_S: f64 = 0.5; //In meters per second