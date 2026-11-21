// FOR IREC ROCKET - CHECK TO ENSURE THESE VALUES ARE CODED:

//Flight-specific parameters
pub const ACCEL_THRESHOLD_GS_SQUARED: f64 = 8.0 * 8.0; //In Gs squared
pub const ALTITUDE_THRESHOLD_M: f64 = 5.0; //In meters
pub const MAIN_DEPLOYMENT_ALTITUDE_M: f64 = 152.4; //In meters (1500 ft)
pub const FLIGHT_DURATION_S: u64 = 300; //In seconds
pub const APOGEE_ERROR_M: f64 = 0.0; //In meters
pub const APOGEE_TIMEOUT_S: u64 = 0; //In seconds
pub const FLIGHT_TIMEOUT_S: u64 = 300; //In seconds
pub const LANDED_VELOCITY_THRESHOLD_M_S: f64 = -1.0; //In meters per second
pub const LANDED_MAX_ALTITUDE_M: f64 = 50.0; //Landing only counts below this altitude above the pad, in meters
pub const THRESHOLD_WINDOW: usize = 50; //Size of ring buffer for apogee/landing detection
pub const THRESHOLD_REQUIRED: usize = 35; //Values meeting threshold in ring buffer to pass the check

//LoRa broadcast config
pub const STANDBY_BROADCAST_HZ: f64 = 2.0; //In Hz
pub const FLIGHT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const DESCENT_BROADCAST_HZ: f64 = 5.0; //In Hz
pub const LANDED_BROADCAST_HZ: f64 = 2.0; //In Hz

//Sampling rate
pub const LOOP_RATE_HZ: f64 = 100.0; //Main loop rate

//USB broadcast config
pub const USB_BROADCAST_HZ: f64 = 10.0; //In Hz
pub const USB_BROADCASTING_ALLOWED: bool = true;
// Send an EkfDebug packet over USB every main loop iteration
pub const EKF_DEBUG_ENABLED: bool = false;
// Also send an EkfDebug packet over LoRa after each LoRa DataState packet.
// Doubles radio airtime, so consider turning off for flights
pub const EKF_DEBUG_RADIO_ENABLED: bool = false;

