
#include "base.h"

vec2f vec2f_add(vec2f a, vec2f b) {
    return (vec2f){ a.x + b.x, a.y + b.y };
}

vec2f vec2f_sub(vec2f a, vec2f b) {
    return (vec2f){ a.x - b.x, a.y - b.y };
}

vec2f vec2f_comp_mul(vec2f a, vec2f b) {
    return (vec2f){ a.x * b.x, a.y * b.y };
}

vec2f vec2f_comp_div(vec2f a, vec2f b) {
    return (vec2f){ a.x / b.x, a.y / b.y };
}

vec2f vec2f_scale(vec2f v, f32 s) {
    return (vec2f){ v.x * s, v.y * s };
}

vec2f vec2f_perp(vec2f v) {
    return (vec2f){ -v.y, v.x };
}

f32 vec2f_cross(vec2f a, vec2f b) {
    return a.x * b.y - a.y * b.x;
}

f32 vec2f_dot(vec2f a, vec2f b) {
    return a.x * b.x + a.y * b.y;
}

f32 vec2f_sqr_dist(vec2f a, vec2f b) {
    return (a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y);
}

f32 vec2f_dist(vec2f a, vec2f b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) +
        (a.y - b.y) * (a.y - b.y));
}

b8 vec2f_eq(vec2f a, vec2f b) {
    return a.x == b.x && a.y == b.y;
}

f32 vec2f_sqr_len(vec2f v) {
    return v.x * v.x + v.y * v.y;
}

f32 vec2f_len(vec2f v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

vec2f vec2f_norm(vec2f v) {
    f32 len = sqrtf(v.x * v.x + v.y * v.y);

    if (len > 1e-8f) {
        return (vec2f){ v.x / len, v.y / len };
    }

    return (vec2f){ 1, 0 };
}

vec3f vec3f_add(vec3f a, vec3f b) {
    return (vec3f){ a.x + b.x, a.y + b.y, a.z + b.z };
}

vec3f vec3f_sub(vec3f a, vec3f b) {
    return (vec3f){ a.x - b.x, a.y - b.y, a.z - b.z };
}

vec3f vec3f_comp_mul(vec3f a, vec3f b) {
    return (vec3f){ a.x * b.x, a.y * b.y, a.z * b.z };
}

vec3f vec3f_comp_div(vec3f a, vec3f b) {
    return (vec3f){ a.x / b.x, a.y / b.y, a.z / b.z };
}

vec3f vec3f_scale(vec3f v, f32 s) {
    return (vec3f){ v.x * s, v.y * s, v.z * s };
}

vec3f vec3f_cross(vec3f a, vec3f b) {
    return (vec3f) {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

f32 vec3f_dot(vec3f a, vec3f b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

f32 vec3f_sqr_dist(vec3f a, vec3f b) {
    return (a.x - b.x) * (a.x - b.x) + 
        (a.y - b.y) * (a.y - b.y) + 
        (a.z - b.z) * (a.z - b.z);
}

f32 vec3f_dist(vec3f a, vec3f b) {
    return sqrtf((a.x - b.x) * (a.x - b.x) + 
        (a.y - b.y) * (a.y - b.y) + 
        (a.z - b.z) * (a.z - b.z));
}

b8 vec3f_eq(vec3f a, vec3f b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

f32 vec3f_sqr_len(vec3f v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
}

f32 vec3f_len(vec3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

vec3f vec3f_norm(vec3f v) {
    f32 len = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);

    if (len > 1e-8f) {
        f32 r = 1.0f / len;
        return (vec3f){ v.x * r, v.y * r, v.z * r };
    }

    return (vec3f){ 1, 0, 0 };
}

quatf quatf_add(quatf a, quatf b) {
    return (quatf){
        .w = a.w + b.w,
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z,
    };
}

quatf quatf_mul(quatf a, quatf b) {
    return (quatf) {
        .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

f32 quatf_dot(quatf a, quatf b) {
    return (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
}

quatf quatf_norm(quatf q) {
    f32 mag = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (mag > 1e-8f) {
        f32 r = 1.0f / mag;

        return (quatf){
            .w = r * q.w,
            .x = r * q.x,
            .y = r * q.y,
            .z = r * q.z,
        };
    }

    return (quatf){ .w = 1.0f };
}

vec3f quatf_rot_vec3f(quatf q, vec3f v) {
    vec3f u = { q.x, q.y, q.z };

    f32 s = q.w;

    vec3f out = vec3f_scale(u, 2.0f * vec3f_dot(u, v));
    out = vec3f_add(out, vec3f_scale(v, s * s - vec3f_dot(u, u)));
    out = vec3f_add(out, vec3f_scale(vec3f_cross(u, v), 2.0f * s));
    return out;
}

void quatf_to_mat3(quatf q, f32 m_out[3 * 3]) {
    m_out[0 * 3 + 0] = 2.0f * (q.w * q.w + q.x * q.x) - 1.0f;
    m_out[0 * 3 + 1] = 2.0f * (q.x * q.y - q.w * q.z);
    m_out[0 * 3 + 2] = 2.0f * (q.x * q.z + q.w * q.y);

    m_out[1 * 3 + 0] = 2.0f * (q.x * q.y + q.w * q.z);
    m_out[1 * 3 + 1] = 2.0f * (q.w * q.w + q.y * q.y) - 1.0f;
    m_out[1 * 3 + 2] = 2.0f * (q.y * q.z - q.w * q.x);

    m_out[2 * 3 + 0] = 2.0f * (q.x * q.z - q.w * q.y);
    m_out[2 * 3 + 1] = 2.0f * (q.y * q.z + q.w * q.x);
    m_out[2 * 3 + 2] = 2.0f * (q.w * q.w + q.z * q.z) - 1.0f;
}

void _mm_nn(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, const f32* A, const f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 k = 0; k < ap_cols; k++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[k + j * ap_cols] * B[i + k * c_cols];
            }
        }
    }
}

void _mm_nt(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, const f32* A, const f32* B, f32* C
) {
    for (u32 k = 0; k < ap_cols; k++) {
        for (u32 i = 0; i < c_cols; i++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[k + j * ap_cols] * B[k + i * ap_cols];
            }
        }
    }
}

void _mm_tn(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, const f32* A, const f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 j = 0; j < c_rows; j++) {
            for (u32 k = 0; k < ap_cols; k++) {
                C[i + j * c_cols] += alpha * A[j + k * c_rows] * B[i + k * c_cols];
            }
        }
    }
}

void _mm_tt(
    u32 c_rows, u32 c_cols, u32 ap_cols,
    f32 alpha, const f32* A, const f32* B, f32* C
) {
    for (u32 i = 0; i < c_cols; i++) {
        for (u32 k = 0; k < ap_cols; k++) {
            for (u32 j = 0; j < c_rows; j++) {
                C[i + j * c_cols] += alpha * A[j + k * c_rows] * B[k + i * ap_cols];
            }
        }
    }
}

void matmul(
    b8 transpose_a, b8 transpose_b,
    u32 c_rows, u32 c_cols, u32 ap_cols, 
    f32 alpha, const f32* A, const f32* B,
    f32 beta, f32* C
) {
    u32 c_size = c_rows * c_cols;

    if (beta == 0.0f) {
        memset(C, 0, c_size * sizeof(f32));
    } else {
        for (u32 i = 0; i < c_size; i++) {
            C[i] *= beta;
        }
    }

    u32 transpose_bits = (u32)((!!transpose_a << 1) | !!transpose_b);
    switch (transpose_bits) {
        case 0b00: _mm_nn(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b01: _mm_nt(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b10: _mm_tn(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        case 0b11: _mm_tt(c_rows, c_cols, ap_cols, alpha, A, B, C); break;
        
        default: break;
    }
}

b32 linear_solve(u32 a_size, u32 b_cols, f32* A, f32* B) {
    // This implementation uses gauss jordan elimination 

    for (u32 i = 0; i < a_size; i++) {
        // Finding the max element
        f32 max_elem = 0.0f;
        u32 max_index = 0;

        for (u32 j = i; j < a_size; j++) {
            f32 abs_elem = ABS(A[i + j * a_size]);

            if (abs_elem > max_elem) {
                max_elem = abs_elem;
                max_index = j;
            }
        }

        if (max_elem <= 1e-8f) { return false; }

        // Pivoting rows if required
        if (max_index != i) {
            // Pivoting A
            for (u32 j = 0; j < a_size; j++) {
                u32 index0 = j + i * a_size;
                u32 index1 = j + max_index * a_size;

                f32 tmp = A[index0];
                A[index0] = A[index1];
                A[index1] = tmp;
            }

            // Pivoting B
            for (u32 j = 0; j < b_cols; j++) {
                u32 index0 = j + i * b_cols;
                u32 index1 = j + max_index * b_cols;

                f32 tmp = B[index0];
                B[index0] = B[index1];
                B[index1] = tmp;
            }
        }

        // Applying the row options
        for (u32 j = i + 1; j < a_size; j++) {
            f32 factor = A[i + j * a_size] / A[i + i * a_size];

            // Applying operation to A
            for (u32 k = i + 1; k < a_size; k++) {
                A[k + j * a_size] -= factor * A[k + i * a_size];
            }

            // Applying operation to B
            for (u32 k = 0; k < b_cols; k++) {
                B[k + j * b_cols] -= factor * B[k + i * b_cols];
            }
        }
    }

    // Final step is to rescale each row of B and add them back up
    // Basically going from ref to rref without actually touching A
    for (i32 i = (i32)a_size - 1; i >= 0; i--) {
        f32 factor = 1.0f / A[(u32)i + (u32)i * a_size];

        // Rescale
        for (u32 j = 0; j < b_cols; j++) {
            B[j + (u32)i * b_cols] *= factor;
        }

        // Add rows
        for (u32 j = 0; j < (u32)i; j++) {
            factor = -A[(u32)i + j * a_size];

            for (u32 k = 0; k < b_cols; k++) {
                B[k + j * b_cols] += factor * B[k + (u32)i * b_cols];
            }
        }
    }

    return true;
}