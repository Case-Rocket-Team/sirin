#include "arm_math.h"

float32_t test_sin(float32_t x) {
    return arm_sin_f32(x);
}

float32_t rotation_matrix_nominal_data[9];

struct State {
    // nominal
    float32_t position_nominal[3];
    float32_t velocity_nominal[3];
    float32_t quaternion_nominal[4];
    arm_matrix_instance_f32 rotation_matrix_nominal;

    float32_t accel_bias_nominal[3];
    float32_t angular_vel_bias_nominal[3];
    float32_t gravity_nominal[3];
};

void update_nominal(
    struct State *state,
    float32_t dt,
    float32_t accel_measurement[3],
    float32_t angular_vel_measurement[3]
) {
    // Section 5.4.1

    float32_t accel_term[3];
    arm_sub_f32(&accel_measurement, &state->accel_bias_nominal, &accel_term, 3);
    arm_mat_vec_mult_f32(&state->rotation_matrix_nominal, &accel_term, &accel_term);
    arm_add_f32(&accel_term, &state->gravity_nominal, &accel_term, 3);

    // Updating position -- 259a
    {
        float32_t vel_term[3];
        arm_scale_f32(&state->velocity_nominal, dt, &vel_term, 3);
        arm_add_f32(&state->position_nominal, vel_term, &state->position_nominal, 3);
        arm_add_f32(&state->position_nominal, accel_term, &state->position_nominal, 3);
        
        float32_t accel_term2[3];
        arm_scale_f32(&accel_term, 0.5 * dt * dt, &accel_term2, 3);
    }

    // Updating velocity -- 259b
    {
        float32_t accel_term2[3];
        arm_scale_f32(&accel_term, dt, &accel_term2, 3);
        arm_add_f32(&state->velocity_nominal, &accel_term2, &state->velocity_nominal, 3);
    }

    // Updating quaternion -- 259c
    {
        // TODO -- check their quaternion impl
        float32_t fac[3];
        arm_sub_f32(&angular_vel_measurement, &state->angular_vel_bias_nominal, fac, 3);
        arm_scale_f32(&fac, dt, &fac, 3);
    }
}