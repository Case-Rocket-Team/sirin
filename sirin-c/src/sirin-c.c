#include "../cmsis-dsp/Include/arm_math.h"
// #include "arm_math.h"

// Following https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf    

// ref. table 3, pg. 52

#define DEBUG 1

#define LOGLVL_DEBUG 1
#define LOGLVL_INFO 2
#define LOGLVL_WARN 3
#define LOGLVL_ERROR 4

#define GRAVITY 9.80665
#define STATE_DIMS 18
#define STATE_MAT_SIZE 324 // 6 3d state vars -> 18 dims -> 18*18 mat
#define EARTH_RADIUS 6378137.0

#define VEL_NOISE 1
#define ANGLES_NOISE 1
#define ACCEL_NOISE 1
#define ANGULAR_VEL_NOISE 1

#define OK 0
#define ERR_INIT_NO_ACCEL 1

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

struct CovarianceMatrixP {
    float32_t data[STATE_MAT_SIZE];
};

extern void sirin_log(uint32_t lvl, char *msg);
extern void sirin_log_f32_array(float32_t *ptr, size_t len);

float64_t pressure_altitude(float64_t pressure_hpa) {
    // TODO
    //return 44307.7 - 11872.4 * powl(pressure_hpa, 0.190284);
    return 0.0;
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
// https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
/*void quaternion_between_vecs(
    float32_t pSrcUnitVecA[3],
    float32_t pSrcUnitVecB[3],
    float32_t pDstQuat[4]
) {
    // todo optimize
    cross_product(pSrcUnitVecA, pSrcUnitVecB, &pDstQuat[1]);
    float32_t dot;
    arm_dot_prod_f32(pSrcUnitVecA, pSrcUnitVecB, 3, &dot);
    pDstQuat[0] = 1.0f + dot;
    arm_quaternion_normalize_f32(pDstQuat, pDstQuat, 1);
}*/

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

/**
 * Assuming src is smaller than dst
 */
void set_block(
    const float32_t *pSrc,
    float32_t *pDst,
    size_t srcWidth,
    size_t dstWidth,
    size_t row,
    size_t col 
) {
    for (size_t y = 0; y < srcWidth; y++) {
        for (size_t x = 0; x < srcWidth; x++) {
            pDst[(row + y) * dstWidth + (col + x)] =
                pSrc[y * srcWidth + x];
        }
    }
}

/**
 * Copies a NxM block from a src matrix to dst matrix
 */
void copy_block(
    const float32_t *pSrc,
    float32_t *pDst,
    size_t blockCols,
    size_t blockRows,
    size_t srcWidth,
    size_t dstWidth,
    size_t row,
    size_t col 
) {
    for (size_t y = 0; y < blockRows; y++) {
        for (size_t x = 0; x < blockCols; x++) {
            pDst[(row + y) * dstWidth + (col + x)] =
                pSrc[(row + y) * srcWidth + (col + x)];
        }
    }
}

void set_3d_identity_block(
    float32_t *pDst,
    size_t dstWidth,
    size_t row,
    size_t col 
) {
    for (size_t i = 0; i < 3; i++) {
        size_t rowDst = row + i;
        size_t colDst = col + i;

        float32_t *pDstI = pDst + rowDst * dstWidth + colDst;
        *pDstI = 1.0f;
    }
}

void set_3d_diagonal_block(
    float32_t *pDst,
    size_t dstWidth,
    size_t row,
    size_t col,
    float32_t value
) {
    for (size_t i = 0; i < 3; i++) {
        size_t rowDst = row + i;
        size_t colDst = col + i;

        float32_t *pDstI = pDst + rowDst * dstWidth + colDst;
        *pDstI = value;
    }
}


// vector times its transpose
// vv^T
void vec_outer_product(
    const float32_t *pSrcVec,
    float32_t *pDstMat
) {
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
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

    if (mag < 0.001) {
        sirin_log(LOGLVL_DEBUG, "Magnitude of decompose_vec is too small, might get NaNs!");
    }

    arm_scale_f32(pSrcVec, 1 / mag, pDstUnitVec, 3);
}

void vec2quaternion(
    float32_t *pSrcVec,
    float32_t *pDstQuaternion
) {
    float32_t mag;
    float32_t axis[3];
    decompose_vec(pSrcVec, &mag, axis);

    if (mag < 1e-9) {
        // Small-angle approximation
        pDstQuaternion[0] = 1.0f;
        pDstQuaternion[1] = 0.5f * pSrcVec[0];
        pDstQuaternion[2] = 0.5f * pSrcVec[1];
        pDstQuaternion[3] = 0.5f * pSrcVec[2];
        return;
    }

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

void correct_from_gps(
    struct NominalState *nominal,
    struct ErrorState *error,
    struct CovarianceMatrixP *cov,
    float32_t *gps_position,
    float32_t *gps_velocity
) {
    // GPS horizontal accuracy: 1.5m
    // GPS vertical accuracy (*1.7, according to google): 2.55m
    // GPS velocity accuracy: 0.05 m/s
    // BETTER: get hAcc and vAcc from the UBX GPS packet
    const int N_MEAS = 6;
    float32_t hAcc = 1.5f, vAcc = 2.55f;
    float32_t sAcc = 0.05f;

    float32_t v_mat_data[N_MEAS * N_MEAS];
    memset(v_mat_data, 0, sizeof(v_mat_data));
    // horizontal position accuracy
    v_mat_data[0] = hAcc * hAcc;
    v_mat_data[7] = hAcc * hAcc;
    // vertical position accuracy
    v_mat_data[14] = vAcc * vAcc;
    // velocity accuracy
    v_mat_data[21] = sAcc * sAcc;
    v_mat_data[28] = sAcc * sAcc;
    v_mat_data[35] = sAcc * sAcc;
    arm_matrix_instance_f32 v_mat = {
        .numCols = 6,
        .numRows = 6,
        .pData = v_mat_data
    };

    // GPS H: 
    // [I 0 0 0 0 0]
    // [0 I 0 0 0 0]
    // float32_t h_mat_data[N_MEAS * STATE_DIMS];
    // set_3d_identity_block(h_mat_data, 18, 0, 0);
    // set_3d_identity_block(h_mat_data, 18, 3, 3);
    // arm_matrix_instance_f32 h_mat = {
    //     .numCols = STATE_DIMS,
    //     .numRows = N_MEAS,
    //     .pData = h_mat_data
    // };

    // EQ 273
    // K = P H^T (H P H^T + V)^-1

    // H P H^T + V
    float32_t s_mat_data[N_MEAS * N_MEAS];
    arm_matrix_instance_f32 s_mat = {
        .numCols = N_MEAS,
        .numRows = N_MEAS,
        .pData = s_mat_data
    };
    // H P H^T = first 6 rows and cols of P
    copy_block(cov->data, s_mat_data, N_MEAS, N_MEAS, STATE_DIMS, N_MEAS, 0, 0);
    arm_mat_add_f32(&s_mat, &v_mat, &s_mat);

    // arm_status status = arm_mat_cholesky_f32(&s_mat, &s_mat);
    // if (status != ARM_MATH_SUCCESS) {
    //     sirin_log(INFO, "GPS update: Cholesky decomposition failed!");
    //     return;
    // }

    arm_mat_inverse_f32(&s_mat, &s_mat);

    // P H^T
    float32_t pht_mat_data[STATE_DIMS * N_MEAS];
    arm_matrix_instance_f32 pht_mat = {
        .numCols = N_MEAS,
        .numRows = STATE_DIMS,
        .pData = pht_mat_data
    };
    copy_block(cov->data, pht_mat_data, N_MEAS, STATE_DIMS, N_MEAS, STATE_DIMS, 0, 0);

    // pht_mat = K
    arm_mat_mult_f32(&pht_mat, &s_mat, &pht_mat);

    // 275 Joseph form
    // P ← (I − KH)P(I − KH)^T + KVK^T

    // I - KH
    float32_t ikh_mat_data[STATE_MAT_SIZE];
    memset(ikh_mat_data, 0, sizeof(ikh_mat_data));
    set_3d_identity_block(ikh_mat_data, STATE_DIMS, 0, 0);
    for (size_t i = 0; i < N_MEAS; i++) {
        for (size_t j = 0; j < STATE_DIMS; j++) {
            ikh_mat_data[i * STATE_DIMS + j] -= pht_mat_data[j * N_MEAS + i];
        }
    }
    arm_matrix_instance_f32 ikh_mat = {
        .numCols = STATE_DIMS,
        .numRows = STATE_DIMS,
        .pData = ikh_mat_data
    };

    arm_matrix_instance_f32 p_mat = {
        .numCols = STATE_DIMS,
        .numRows = STATE_DIMS,
        .pData = cov->data
    };

    // (I - KH) P
    float32_t temp_mat_data[STATE_MAT_SIZE];
    arm_matrix_instance_f32 temp_mat = {
        .numCols = STATE_DIMS,
        .numRows = STATE_DIMS,
        .pData = temp_mat_data
    };
    arm_mat_mult_f32(&ikh_mat, &p_mat, &temp_mat);

    // (I - KH)^T
    arm_mat_trans_f32(&ikh_mat, &ikh_mat);

    // (I - KH) P (I - KH)^T
    arm_mat_mult_f32(&temp_mat, &ikh_mat, &temp_mat);

    // K V K^T
    float32_t kvt_mat_data[STATE_MAT_SIZE];
    arm_matrix_instance_f32 kvt_mat = {
        .numCols = STATE_DIMS,
        .numRows = STATE_DIMS,
        .pData = kvt_mat_data
    };
    arm_mat_mult_f32(&pht_mat, &v_mat, &kvt_mat);
    arm_mat_trans_f32(&kvt_mat, &kvt_mat);
    arm_mat_mult_f32(&kvt_mat, &pht_mat, &kvt_mat);

    // P ← (I - KH)P(I - KH)^T + KVK^T
    arm_add_f32(temp_mat_data, kvt_mat_data, cov->data, STATE_MAT_SIZE);
}

uint32_t init_with_imu(
    struct NominalState *nominal,
    struct ErrorState *error,
    float32_t *accel_measurement,
    float32_t *angular_vel_measurement,
    float32_t *magnetometer_measurement
) {
    float32_t accel_unit[3];
    float32_t accel_mag;
    decompose_vec(accel_measurement, &accel_mag, accel_unit);

    if (accel_mag < 1e-3) {
        return ERR_INIT_NO_ACCEL;
    }

    // project magnetometer_measurement onto accel_unit (aka gravity)
    float32_t dot;
    arm_dot_prod_f32(accel_unit, magnetometer_measurement, 3, &dot);
    
    float32_t proj[3];
    arm_scale_f32(accel_unit, dot, proj, 3);

    float32_t north_unnorm[3];
    arm_sub_f32(magnetometer_measurement, proj, north_unnorm, 3);

    float32_t north_unit[3];
    float32_t norm_unnorm_mag;
    decompose_vec(north_unnorm, &norm_unnorm_mag, north_unit);

    float32_t east[3];
    cross_product(accel_unit, north_unit, east);

    #if DEBUG
        float32_t east_mag;
        float32_t east_unit[3];
        decompose_vec(east, &east_mag, east_unit);

        if (fabs(east_mag - 1.0) > 0.01f) {
            sirin_log(LOGLVL_ERROR, "East vector should be a unit vector because it is the cross product of two orthogonal unit vectors, but it is not a unit vector!");
        }
    #endif

    float32_t rot_mat[9];
    // first column is -g
    rot_mat[0] = -accel_unit[0];
    rot_mat[3] = -accel_unit[1];
    rot_mat[6] = -accel_unit[2];

    // second column is east
    rot_mat[1] = east[0];
    rot_mat[4] = east[1];
    rot_mat[7] = east[2];

    // third column is north
    rot_mat[2] = north_unit[0];
    rot_mat[5] = north_unit[1];
    rot_mat[8] = north_unit[2];

    arm_rotation2quaternion_f32(rot_mat, nominal->rot_quaternion, 1);

    nominal->pos[0] = EARTH_RADIUS;

    #if DEBUG
        // Check: X × Y should equal Z
        // Not sure why this always throws even when it shouldnt
        /*float32_t x_cross_y[3];
        cross_product(&rot_mat[0], &rot_mat[1], x_cross_y);

        float32_t dot_with_z;
        arm_dot_prod_f32(x_cross_y, &rot_mat[2], 3, &dot_with_z);

        if (fabs(dot_with_z - 1.0f) > 0.01f) {
            sirin_log_f32_array(rot_mat, 9);
            sirin_log(LOGLVL_ERROR, "Rotation matrix not right-handed!");
        }*/
    #endif

    return OK;
}

void update_with_imu(
    struct NominalState *nominal,
    struct ErrorState *error,
    struct CovarianceMatrixP *cov,
    float32_t dt,
    float32_t *accel_measurement,
    float32_t *angular_vel_measurement
) {
    sirin_log(LOGLVL_DEBUG, "Update with IMU: Starting nominal updates");

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

    // g should always point towards center of the earth. 
    float32_t pos_mag;
    float32_t pos_unit[3];
    // TODO: should I be using the composite pos (from incorp both nominal and error) or just the nominal?
    decompose_vec(nominal->pos, &pos_mag, pos_unit);

    float32_t gravity[3];
    //float32_t gravity_mag = gravity_at_altitude(nominal->pos[2]);
    arm_scale_f32(pos_unit, -GRAVITY, gravity, 3);
    arm_add_f32(accel_term, gravity, accel_term, 3);
    memcpy(nominal->accel, accel_term, sizeof(nominal->accel));

    // Updating position -- 259a
    {
        float32_t vel_term[3];
        arm_scale_f32(nominal->vel, dt, vel_term, 3);
        arm_add_f32(nominal->pos, vel_term, nominal->pos, 3);
        
        float32_t accel_term2[3];
        arm_scale_f32(accel_term, 0.5 * dt * dt, accel_term2, 3);
        arm_add_f32(nominal->pos, accel_term2, nominal->pos, 3);
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

    sirin_log(LOGLVL_DEBUG, "Error updates");

    // ERROR UPDATES

    // Update pos err -- 260a
    {
        float32_t vel_term[3];
        arm_scale_f32(error->vel, dt, vel_term, 3);
        arm_add_f32(error->pos, vel_term, error->pos, 3);
    }

    // this will be used later
    float32_t accel_mat_for_jacobian_data[9];
    memset(accel_mat_for_jacobian_data, 0, sizeof(accel_mat_for_jacobian_data));
    arm_matrix_instance_f32 accel_mat_for_jacobian_mat = {
        .numCols = 3,
        .numRows = 3,
        .pData = accel_mat_for_jacobian_data
    };

    // Update velocity error -- 260b
    {
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
            
            memcpy(accel_mat_for_jacobian_data, accel_mat_data, sizeof(accel_mat_for_jacobian_data));
            arm_mat_scale_f32(&accel_mat_for_jacobian_mat, -dt, &accel_mat_for_jacobian_mat);

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

    float32_t angular_mat_for_jacobian_data[9];
    memset(angular_mat_for_jacobian_data, 0, sizeof(angular_mat_for_jacobian_data));
    arm_matrix_instance_f32 angular_mat_for_jacobian_mat = {
        .numCols = 3,
        .numRows = 3,
        .pData = angular_mat_for_jacobian_data
    };

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
            memcpy(angular_mat_for_jacobian_data, angles_rot_mat_trans_data, sizeof(angles_rot_mat_trans_data));

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

    sirin_log(LOGLVL_DEBUG, "Covariance matrix update");
    sirin_log(LOGLVL_DEBUG, "Set up Jacobian");

    // Update covariance matrix P -- 268
    float32_t jacobian_f[STATE_MAT_SIZE];
    memset(jacobian_f, 0, sizeof(jacobian_f));

    sirin_log(LOGLVL_DEBUG, "Row 1");

    // Row #1
    set_3d_identity_block(jacobian_f, STATE_DIMS, 0, 0);
    set_3d_diagonal_block(jacobian_f, STATE_DIMS, 0, 3, dt);

    sirin_log(LOGLVL_DEBUG, "Row 2");

    // Row #2
    set_3d_identity_block(jacobian_f, STATE_DIMS, 3, 3);
    set_block(accel_mat_for_jacobian_data, jacobian_f, 3, STATE_DIMS, 3, 6);
    set_3d_diagonal_block(jacobian_f, STATE_DIMS, 3, 15, dt);

    sirin_log(LOGLVL_DEBUG, "Row 3");

    // Row #3
    set_block(angular_mat_for_jacobian_data, jacobian_f, 3, 3, 6, 6);
    set_3d_diagonal_block(jacobian_f, STATE_DIMS, 6, 12, -dt);

    sirin_log(LOGLVL_DEBUG, "Row 4");

    // Row #4
    set_3d_identity_block(jacobian_f, STATE_DIMS, 9, 9);

    sirin_log(LOGLVL_DEBUG, "Row 5");

    // Row #5
    set_3d_identity_block(jacobian_f, STATE_DIMS, 12, 12);

    sirin_log(LOGLVL_DEBUG, "Row 6");

    // Row #6
    set_3d_identity_block(jacobian_f, STATE_DIMS, 15, 15);
    
    sirin_log(LOGLVL_DEBUG, "Multiply covariance by Jacobian of state pt. 1");

    arm_matrix_instance_f32 cov_mat = {
        .numCols = 18,
        .numRows = 18,
        .pData = cov->data
    };

    float32_t cov_mat_copy_data[STATE_MAT_SIZE];
    arm_matrix_instance_f32 cov_mat_copy = {
        .numCols = 18,
        .numRows = 18,
        .pData = cov_mat_copy_data
    };

    arm_matrix_instance_f32 jacobian_f_mat = {
        .numCols = 18,
        .numRows = 18,
        .pData = jacobian_f
    };

    arm_mat_mult_f32(&jacobian_f_mat, &cov_mat, &cov_mat_copy);

    sirin_log(LOGLVL_DEBUG, "Multiply covariance by Jacobian of state pt. 2");
    
    float32_t jacobian_f_trans_data[STATE_MAT_SIZE];
    arm_matrix_instance_f32 jacobian_f_trans = {
        .numCols = 18,
        .numRows = 18,
        .pData = jacobian_f_trans_data
    };

    sirin_log(LOGLVL_DEBUG, "Jacobian transpose");

    arm_mat_trans_f32(&jacobian_f_mat, &jacobian_f_trans);

    sirin_log(LOGLVL_DEBUG, "Multiply by Jacobian transpose");

    arm_mat_mult_f32(&cov_mat_copy, &jacobian_f_trans, &cov_mat);

    sirin_log(LOGLVL_DEBUG, "Add covariance from uncertainty");

    float32_t FQFt_data[STATE_MAT_SIZE];
    memset(FQFt_data, 0, sizeof(FQFt_data));
    arm_matrix_instance_f32 FQFt = {
        .numCols = 18,
        .numRows = 18,
        .pData = FQFt_data
    };

    set_3d_diagonal_block(FQFt_data, 18, 3, 3, VEL_NOISE);
    set_3d_diagonal_block(FQFt_data, 18, 6, 6, ANGLES_NOISE);
    set_3d_diagonal_block(FQFt_data, 18, 9, 9, ACCEL_NOISE);
    set_3d_diagonal_block(FQFt_data, 18, 12, 12, ANGULAR_VEL_NOISE);

    arm_mat_add_f32(&FQFt, &cov_mat, &cov_mat);
}