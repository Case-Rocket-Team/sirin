/*

File base.h
-----------

Defines shared types and helper functions used by all the airbrake modules.

*/

#ifndef BASE_H
#define BASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#if defined(_WIN32)
#   define PLATFORM_WIN32
#elif defined(__linux__)
#   define PLATFORM_LINUX
#endif
// TODO: MCU platform 

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define CLAMP(x, a, b) (MIN((b), MAX((x), (a))))
#define ABS(n) ((n) < 0 ? -(n) : (n))
#define SIGN(n) ((n) < 0 ? -1 : 1)

// Signed integers
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;

// Unsigned integers
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#ifndef PLATFORM_MCU
typedef int64_t i64;
typedef uint64_t u64;
#endif

// Booleans
typedef i8 b8;
typedef i32 b32;

// Floating point
typedef float f32;

typedef struct { f32 x, y; } vec2f;
typedef struct { f32 x, y, z; } vec3f;

typedef struct { f32 w, x, y, z; } quatf;

typedef struct {
    u8* str;
    u32 size;
} string8;

vec2f vec2f_add(vec2f a, vec2f b);
vec2f vec2f_sub(vec2f a, vec2f b);
vec2f vec2f_comp_mul(vec2f a, vec2f b);
vec2f vec2f_comp_div(vec2f a, vec2f b);
vec2f vec2f_scale(vec2f v, f32 s);
vec2f vec2f_perp(vec2f v);
f32 vec2f_cross(vec2f a, vec2f b);
f32 vec2f_dot(vec2f a, vec2f b);
f32 vec2f_sqr_dist(vec2f a, vec2f b);
f32 vec2f_dist(vec2f a, vec2f b);
b8 vec2f_eq(vec2f a, vec2f b);
f32 vec2f_sqr_len(vec2f v);
f32 vec2f_len(vec2f v);
vec2f vec2f_norm(vec2f v);

vec3f vec3f_add(vec3f a, vec3f b);
vec3f vec3f_sub(vec3f a, vec3f b);
vec3f vec3f_comp_mul(vec3f a, vec3f b);
vec3f vec3f_comp_div(vec3f a, vec3f b);
vec3f vec3f_scale(vec3f v, f32 s);
vec3f vec3f_cross(vec3f a, vec3f b);
f32 vec3f_dot(vec3f a, vec3f b);
f32 vec3f_sqr_dist(vec3f a, vec3f b);
f32 vec3f_dist(vec3f a, vec3f b);
b8 vec3f_eq(vec3f a, vec3f b);
f32 vec3f_sqr_len(vec3f v);
f32 vec3f_len(vec3f v);
vec3f vec3f_norm(vec3f v);

quatf quatf_add(quatf a, quatf b);
quatf quatf_mul(quatf a, quatf b);
f32 quatf_dot(quatf a, quatf b);
quatf quatf_norm(quatf q);

// Performs and optimized version of the operation qvq*
// q must be a unit quaternion
vec3f quatf_rot_vec3f(quatf q, vec3f v);

// The quaternion q must be normalized for this
void quatf_to_mat3(quatf q, f32 m_out[3 * 3]);

/*
This is essentially just a gemm

Computes C = alpha * A' * B' + beta * C

where A' = transpose_a ? A**T : A
and   B' = transpose_b ? B**T : B

A' must be (c_rows, ap_cols)
B' must be (ap_cols, c_cols)
C  must be (c_rows, c_cols)

A, B, and C must be stored row major 
A, B, and C must be non-overlapping
*/
void matmul(
    b8 transpose_a, b8 transpose_b,
    u32 c_rows, u32 c_cols, u32 ap_cols, 
    f32 alpha, const f32* A, const f32* B,
    f32 beta, f32* C
);

/*
Somewhat modeled after LAPACK gesv

Sovles a linear system A * X = B

A must be (a_size, a_size)
B must be (a_size, b_cols)

Returns true if the linear system is consistent, false otherwise
Upon exit, B conatins the solution to the system (if it is consistent)

A is modified by the function
*/
b32 linear_solve(u32 a_size, u32 b_cols, f32* A, f32* B);

#ifdef __cplusplus
}
#endif

#endif // BASE_H