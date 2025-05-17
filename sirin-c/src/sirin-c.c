#include "arm_math.h"

float32_t test_sin(float32_t x) {
    return arm_sin_f32(x);
}

struct NominalState {
    float32_t pos[3];
    float32_t vel[3];
    float32_t accel[3];

    float32_t rot_quaternion[4];

    float32_t accel_bias[3];
    float32_t angular_vel_bias[3];
    float32_t gravity[3];
};

// eq. 101
void rot_axis2quaternion(
    float32_t *out_quaternion,

    /// Normalized axis
    float32_t *axis,

    // rad
    float32_t angle
) {
    out_quaternion[0] = arm_cos_f32(angle / 2);
    float32_t sin = arm_sin_f32(angle / 2);
    out_quaternion[1] = axis[0] * sin;
    out_quaternion[2] = axis[1] * sin;
    out_quaternion[3] = axis[2] * sin;
}

void vec2quaternion(
    float32_t *out_quaternion,
    float32_t *vec
) {
    float32_t mag;
    arm_dot_prod_f32(vec, vec, 3, &mag);
    arm_sqrt_f32(mag, &mag);

    float32_t axis[3];
    arm_scale_f32(vec, 1 / mag, axis, 3);

    rot_axis2quaternion(out_quaternion, axis, mag);
}

void update_nominal(
    struct NominalState *state,
    float32_t dt,
    float32_t *accel_measurement,
    float32_t *angular_vel_measurement
) {
    // Following https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf    

    memcpy(state->accel, accel_measurement, 3*4);

    // Section 5.4.1
    // Create rotation matrix from quaternion
    float32_t rot_matrix_data[9];
    arm_matrix_instance_f32 rot_matrix = {
        .numCols = 3,
        .numRows = 3,
        .pData = rot_matrix_data
    };
    arm_quaternion2rotation_f32(state->rot_quaternion, rot_matrix.pData, 1);

    // common accel term
    float32_t accel_term[3];
    arm_sub_f32(accel_measurement, state->accel_bias, accel_term, 3);
    arm_mat_vec_mult_f32(&rot_matrix, accel_term, accel_term);
    arm_add_f32(accel_term, state->gravity, accel_term, 3);

    // Updating position -- 259a
    {
        float32_t vel_term[3];
        arm_scale_f32(state->vel, dt, vel_term, 3);
        arm_add_f32(state->pos, vel_term, state->pos, 3);
        arm_add_f32(state->pos, accel_term, state->pos, 3);
        
        float32_t accel_term2[3];
        arm_scale_f32(accel_term, 0.5 * dt * dt, accel_term2, 3);
    }

    // Updating velocity -- 259b
    {
        float32_t accel_term2[3];
        arm_scale_f32(accel_term, dt, accel_term2, 3);
        arm_add_f32(state->vel, accel_term2, state->vel, 3);
    }

    // Updating quaternion -- 259c
    {
        float32_t vec[3];
        arm_sub_f32(angular_vel_measurement, state->angular_vel_bias, vec, 3);
        arm_scale_f32(vec, dt, vec, 3);
        
        float32_t rotate[4];
        vec2quaternion(rotate, vec);
        arm_quaternion_product_single_f32(state->rot_quaternion, rotate, state->rot_quaternion);

        // renormalize due to floating point errors
        arm_quaternion_normalize_f32(state->rot_quaternion, state->rot_quaternion, 1);
    }
}