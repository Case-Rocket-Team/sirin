// FOR IREC ROCKET - CHECK TO ENSURE THESE VALUES ARE CODED:

//Flight-specific parameters
pub const ACCEL_THRESHOLD_GS_SQUARED: f64 = 3.0 * 3.0; //In Gs squared
pub const ALTITUDE_THRESHOLD_M: f64 = 3.0; //In meters
pub const MAIN_DEPLOYMENT_ALTITUDE_M: f64 = 457.2; //In meters (1500 ft)
pub const FLIGHT_DURATION_S: u64 = 10; //In seconds
pub const APOGEE_ERROR_M: f64 = 1.0; //In meters
pub const APOGEE_TIMEOUT_S: u64 = 0; //In seconds
pub const FLIGHT_TIMEOUT_S: u64 = 10; //In seconds
pub const LANDED_VELOCITY_THRESHOLD_M_S: f64 = -0.5; //In meters per second
pub const THRESHOLD_WINDOW: usize = 10; //Size of ring buffer for apogee/landing detection
pub const THRESHOLD_REQUIRED: usize = 8; //Values meeting threshold in ring buffer to pass the check

//LoRa broadcast config
pub const STANDBY_BROADCAST_HZ: f64 = 0.5; //In Hz
pub const FLIGHT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const DESCENT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const LANDED_BROADCAST_HZ: f64 = 0.025; //In Hz

//Sampling rate
pub const LOOP_RATE_HZ: f64 = 10.0; //Main loop rate

//USB broadcast config
pub const USB_BROADCAST_HZ: f64 = 4.0; //In Hz
pub const USB_BROADCASTING_ALLOWED: bool = true;

