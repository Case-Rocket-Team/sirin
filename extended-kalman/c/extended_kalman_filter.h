/*

File extended_kalman_filter.h
-----------------------

This is an error state extended kalman filter that tracks rocket's attitude as
a quaternion. It also tracks position, velocity, and accelerometer and 
gyroscope bias.

Most of this implementation was based off of the following article by Matthew
Hampsey: https://matthewhampsey.github.io/blog/2020/07/18/mekf

*/

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "base/base.h"

#define EKF_STATE_DIM 18
#define EKF_CONTROL_DIM 6
#define EKF_MAX_MEASURE_DIM 3

typedef struct {
    // Stored such that left multiplication by attitude goes from body frame
    // to world frame
    quatf attitude;

    // Velocity is in the world's frame of reference
    vec3f vel_fps;

    // Position is in the world's frame of reference
    vec3f pos_ft;

    vec3f gyro_bias_radps; // radians / sec
    vec3f accel_bias_fps2;
    vec3f magn_bias_gauss;
} ekf_nominal_state;

typedef union {
    struct {
        vec3f accel_fps2;
        vec3f gyro_radps;
    };

    f32 v[EKF_CONTROL_DIM];
} ekf_control_input;

static_assert(sizeof(ekf_control_input) == sizeof(f32) * EKF_CONTROL_DIM);

typedef struct {
    f32 accel_var_f2ps4;
    f32 accel_bias_var_f2ps4;

    f32 gyro_var_rad2ps2;
    f32 gyro_bias_var_rad2ps2;

    f32 baro_var_ft2;

    vec3f world_magn_north_guass;
    f32 magn_covar_gauss2[3 * 3];
    f32 magn_bias_var_gauss2;

    f32 gnss_covar_ft2[3 * 3];
} ekf_settings;

typedef struct {
    ekf_nominal_state nominal_state;

    ekf_settings settings;
    
    // Timestamp of the current state estimate
    u32 state_time_us;

    // Most recent control input, cached so that we can predict to the exact
    // timestamp needed during a given update/prediction step
    ekf_control_input control_input;

    // Covariance for the *error* state
    f32 state_covar[EKF_STATE_DIM * EKF_STATE_DIM];
} extended_kalman_filter;

// Initializes the kalman filter to zero state and the given settings.
// You can just create the `extended_kalman_filter` structure if you would like
// more control, but this ensures you start with a valid attitude quaternion.
void ekf_init(
    extended_kalman_filter* ekf,
    const ekf_settings* settings,
    u32 timestamp_us
);

// Call with IMU data
void ekf_inject_imu(
    extended_kalman_filter* ekf,
    vec3f accel_fps2,
    vec3f gyro_radps,
    u32 timestamp_us
);

// Call with new barometeric altitude data
void ekf_inject_baro(
    extended_kalman_filter* ekf,
    f32 altitude_ft,
    u32 timestamp_us
);

// Call with new magnetometer data
void ekf_inject_magn(
    extended_kalman_filter* ekf,
    vec3f magn_north_gauss,
    u32 timestamp_us
);

// Call with new GPS data
void ekf_inject_gnss(
    extended_kalman_filter* ekf,
    vec3f pos_ft,
    u32 timestamp_us
);

#ifdef __cplusplus
}
#endif

#endif // EXTENDED_KALMAN_FILTER_H