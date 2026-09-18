//! Flight parameters for the nosecone e-bay.
//!
//! These are the values used for the IREC rocket. Do not push code with
//! these values significantly changed without checking with the team.

/// Launch detection: squared acceleration magnitude (in Gs squared) above
/// which the board enters Flight mode. 10 G * 10 G.
pub const ACCEL_THRESHOLD_GS_SQUARED: f64 = 10.0 * 10.0;

/// Launch detection: altitude above the pad (meters) above which the board
/// enters Flight mode.
pub const ALTITUDE_THRESHOLD_M: f64 = 20.0;

/// Altitude above the pad (meters) below which, after apogee, the main
/// parachute is deployed. 457.2 m = 1500 ft.
pub const MAIN_DEPLOYMENT_ALTITUDE_M: f64 = 457.2;

/// Seconds after launch detection at which the board leaves Flight mode
/// and enters Landed mode (stopping flash logging).
pub const FLIGHT_DURATION_S: u64 = 600;

/// Apogee detection margin (meters): apogee is declared once the running
/// maximum altitude exceeds the current altitude by more than this.
pub const APOGEE_ERROR_M: f64 = 1.0;

/// Minimum seconds after launch before the apogee parachute may fire.
/// Guards against a false early apogee detection.
pub const APOGEE_TIMEOUT_S: u64 = 25;
