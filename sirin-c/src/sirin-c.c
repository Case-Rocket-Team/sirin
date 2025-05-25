#include "arm_math.h"


// Following https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf    

// ref. table 3, pg. 52
struct NominalState {
    float32_t pos[3];
    float32_t vel[3];
    float32_t accel[3];

    float32_t rot_quaternion[4];

    float32_t accel_bias[3];
    float32_t angular_vel_bias[3];
    //float32_t gravity[3];
};

struct ErrorState {
    float32_t pos[3];
    float32_t vel[3];
    float32_t accel[3];

    float32_t accel_bias[3];
    float32_t angular_vel_bias[3];
    //float32_t gravity[3];

    float32_t angles_vector[3]; 
};

#define GRAVITY 9.80665

float32_t pressure_altitude(float64_t pressure_hpa) {
    // TODO
    //return 44307.7 - 11872.4 * powl(pressure_hpa, 0.190284);
    return 1.0;
}

float32_t gravity_at_altitude(float32_t altitude_m) {
    float32_t term = (6371008.77 / (6371008.77 + altitude_m));
    return GRAVITY * term * term;
}

// make sure pDst is not pointing to the same array as pSrcA or pSrcB
void cross_product(
    const float32_t pSrcA[3],
    const float32_t pSrcB[3],
    float32_t pDst[3]
) {
    pDst[0] = pSrcA[1] * pSrcB[2] - pSrcA[2] * pSrcB[1];
    pDst[1] = - pSrcA[0] * pSrcB[2] + pSrcA[2] * pSrcB[0];
    pDst[2] = pSrcA[0] * pSrcB[1] - pSrcA[1] * pSrcB[0];
}

// Finds the quaternion that rotates unit vec A into B.
void quaternion_between_vecs(
    float32_t pSrcUnitVecA[3],
    float32_t pSrcUnitVecB[3],
    float32_t pDstQuat[4]
) {
    // todo optimize
    cross_product(pSrcUnitVecA, pSrcUnitVecB, &pDstQuat[1]);
    pDstQuat[0] = 2.0f;
    arm_quaternion_normalize_f32(pDstQuat, pDstQuat, 1);
}

// this is basically the same as a cross product, ref. eq. 20
// TODO: see if this is included in CMSIS somewhere (I couldn't
// find it in a cursory search).
void skew_mat(
    float32_t pSrc[3],
    float32_t pDst[9]
) {
    // row 1
    pDst[0] = 0;
    pDst[1] = -pSrc[2];
    pDst[2] = pSrc[1];

    // row 2
    pDst[3] = pSrc[2];
    pDst[4] = 0;
    pDst[5] = -pSrc[0];

    // row 3
    pDst[6] = -pSrc[1];
    pDst[7] = pSrc[0];
    pDst[8] = 0;
}


// vector times its transpose
// vv^T
void vec_outer_product(
    const float32_t *pSrcVec,
    float32_t *pDstMat
) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            pDstMat[i*3 + j] = pSrcVec[i] * pSrcVec[j];
        }
    }
}

// eq. 101
void rot_axis2quaternion(
    /// Normalized axis
    float32_t *axis,

    // rad
    float32_t angle,

    float32_t *out_quaternion
) {
    out_quaternion[0] = arm_cos_f32(angle / 2);
    float32_t sin = arm_sin_f32(angle / 2);
    out_quaternion[1] = axis[0] * sin;
    out_quaternion[2] = axis[1] * sin;
    out_quaternion[3] = axis[2] * sin;
}

void decompose_vec(
    float32_t *pSrcVec,
    float32_t *pDstMag,
    float32_t *pDstUnitVec
) {
    float32_t mag;
    arm_dot_prod_f32(pSrcVec, pSrcVec, 3, &mag);
    arm_sqrt_f32(mag, &mag);
    *pDstMag = mag;

    arm_scale_f32(pSrcVec, 1 / mag, pDstUnitVec, 3);
}

void vec2quaternion(
    float32_t *pSrcVec,
    float32_t *pDstQuaternion
) {
    float32_t mag;
    float32_t axis[3];
    decompose_vec(pSrcVec, &mag, axis);

    rot_axis2quaternion(axis, mag, pDstQuaternion);
}

void vec2rot_matrix(
    float32_t *pSrcVec,
    float32_t *pDstRotMatrix
) {
    float32_t mag;
    float32_t axis[3];
    decompose_vec(pSrcVec, &mag, axis);

    // eq. 78
    // it would be nice to use the sin_cos function here, but for some reason
    // those take input in degrees.

    float32_t cos = arm_cos_f32(mag);
    float32_t sin = arm_sin_f32(mag);

    for (int x = 0; x < 3; x++) {
        for (int y = 0; y < 3; y++) {
            if (x == y) {
                pDstRotMatrix[y*3 + x] = cos;
            } else {
                pDstRotMatrix[y*3 + x] = 0.0;
            }
        }
    }

    {
        float32_t term[9];
        skew_mat(axis, term);
        arm_scale_f32(term, sin, term, 9);
        arm_add_f32(pDstRotMatrix, term, pDstRotMatrix, 9);
    }

    {
        float32_t term[9];
        vec_outer_product(axis, term);
        arm_scale_f32(term, 1.0 - cos, term, 9);
        arm_add_f32(pDstRotMatrix, term, pDstRotMatrix, 9);
    }
}

void init_with_imu(
    struct NominalState *nominal,
    struct ErrorState *error,
    float32_t *accel_measurement,
    float32_t *angular_vel_measurement 
) {
    // Take the current accel vector as gravity and set the rest as the accel bias.
    // Obviously this won't be accurate but the filter can correct those errors.

    float32_t accel_mag;
    float32_t accel_unit[3];
    decompose_vec(nominal->accel, &accel_mag, accel_unit);

    float32_t gravity[3];
    arm_scale_f32(accel_unit, GRAVITY, gravity, 3);


}

void update_with_imu(
    struct NominalState *nominal,
    struct ErrorState *error,
    float32_t dt,
    float32_t *accel_measurement,
    float32_t *angular_vel_measurement
) {

    // NOMINAL UPDATES
    memcpy(nominal->accel, accel_measurement, 3*4);

    // Section 5.4.1
    // Create rotation matrix from quaternion
    float32_t rot_matrix_data[9];
    arm_matrix_instance_f32 rot_mat = {
        .numCols = 3,
        .numRows = 3,
        .pData = rot_matrix_data
    };
    arm_quaternion2rotation_f32(nominal->rot_quaternion, rot_mat.pData, 1);

    float32_t rot_matrix_data_trans[9];
    arm_matrix_instance_f32 rot_mat_trans = {
        .numCols = 3,
        .numRows = 3,
        .pData = rot_matrix_data_trans
    };
    arm_mat_trans_f32(&rot_mat, &rot_mat_trans);

    // common accel term
    float32_t accel_term[3];
    arm_sub_f32(accel_measurement, nominal->accel_bias, accel_term, 3);
    arm_mat_vec_mult_f32(&rot_mat, accel_term, accel_term);
    //arm_add_f32(accel_term, nominal->gravity, accel_term, 3);
    accel_term[2] -= gravity_at_altitude(nominal->pos[2]);

    // Updating position -- 259a
    {
        float32_t vel_term[3];
        arm_scale_f32(nominal->vel, dt, vel_term, 3);
        arm_add_f32(nominal->pos, vel_term, nominal->pos, 3);
        arm_add_f32(nominal->pos, accel_term, nominal->pos, 3);
        
        float32_t accel_term2[3];
        arm_scale_f32(accel_term, 0.5 * dt * dt, accel_term2, 3);
    }

    // Updating velocity -- 259b
    {
        float32_t accel_term2[3];
        arm_scale_f32(accel_term, dt, accel_term2, 3);
        arm_add_f32(nominal->vel, accel_term2, nominal->vel, 3);
    }

    // Updating quaternion -- 259c
    {
        float32_t vec[3];
        arm_sub_f32(angular_vel_measurement, nominal->angular_vel_bias, vec, 3);
        arm_scale_f32(vec, dt, vec, 3);
        
        float32_t rotate[4];
        vec2quaternion(vec, rotate);
        arm_quaternion_product_single_f32(nominal->rot_quaternion, rotate, nominal->rot_quaternion);

        // renormalize due to floating point errors
        arm_quaternion_normalize_f32(nominal->rot_quaternion, nominal->rot_quaternion, 1);
    }

    // ERROR UPDATES

    // Update pos err -- 260a
    {
        float32_t vel_term[3];
        arm_scale_f32(error->vel, dt, vel_term, 3);
        arm_add_f32(error->pos, vel_term, error->pos, 3);
    }

    // Update velocity error -- 260b
    {
        // (R [a_m - a_b]_times dTheta - R da_b - dg) dt
        float32_t deterministic_term[3];

        // R [a_m - a_b]_times dTheta
        {
            float32_t accel[3];

            // a_m - a_b
            arm_sub_f32(accel_measurement, nominal->accel_bias, accel, 3);

            float32_t accel_mat_data_unrotated[9];
            arm_matrix_instance_f32 accel_mat_unrotated = {
                .numCols = 3,
                .numRows = 3,
                .pData = accel_mat_data_unrotated
            };

            float32_t accel_mat_data[9];
            arm_matrix_instance_f32 accel_mat = {
                .numCols = 3,
                .numRows = 3,
                .pData = accel_mat_data
            };

            // [a_m - a_b]_times
            skew_mat(accel, accel_mat_unrotated.pData);

            // R [a_m - a_b]_times
            arm_mat_mult_f32(&rot_mat, &accel_mat_unrotated, &accel_mat);

            // R [a_m - a_b]_times dTheta
            arm_mat_vec_mult_f32(&accel_mat, error->angles_vector, deterministic_term);
        }
        
        {
            float32_t bias_term[3];
            arm_mat_vec_mult_f32(&rot_mat, error->accel_bias, bias_term);
            arm_add_f32(deterministic_term, bias_term, deterministic_term, 3);
        }

        //arm_sub_f32(deterministic_term, error->gravity, deterministic_term, 3);
        arm_scale_f32(deterministic_term, dt, deterministic_term, 3);

        // Now deterministic_term is
        // (R [a_m - a_b]_times dTheta + R da_b - dg)dt
        arm_sub_f32(error->vel, deterministic_term, error->vel, 3);

        // TODO: stochastic term
    }

    // Update angles error -- 260c
    {
        // first term
        {
            float32_t angular_vel_term[3];
            arm_sub_f32(angular_vel_measurement, nominal->angular_vel_bias, angular_vel_term, 3);
            arm_scale_f32(angular_vel_term, dt, angular_vel_term, 3);

            float32_t angles_rot_mat_data[9];
            arm_matrix_instance_f32 angles_rot_mat = {
                .numCols = 3,
                .numRows = 3,
                .pData = angles_rot_mat_data
            };

            float32_t angles_rot_mat_trans_data[9];
            arm_matrix_instance_f32 angles_rot_mat_trans = {
                .numCols = 3,
                .numRows = 3,
                .pData = angles_rot_mat_trans_data
            };
            vec2rot_matrix(angular_vel_term, angles_rot_mat_data);
            arm_mat_trans_f32(&angles_rot_mat, &angles_rot_mat_trans);

            float32_t angles_term[3];
            arm_mat_vec_mult_f32(&angles_rot_mat, error->angles_vector, angles_term);
            memcpy(error->angles_vector, angles_term, sizeof(angles_term));
        }
        
        // second term
        {
            float32_t bias_term[3];
            arm_scale_f32(error->angular_vel_bias, dt, bias_term, 3);
            arm_sub_f32(error->angles_vector, bias_term, error->angles_vector, 3);
        }

        // TODO: stochastic term
    }
}