/**
 * @file sf_math.h
 * @brief Math utilities for sensor fusion module
 * 
 * Provides wrappers around CMSIS-DSP ARM math functions and common
 * mathematical operations needed by the Kalman filter.
 * 
 * Adapted from Crazyflie firmware cf_math.h
 * Original Copyright (C) 2018-2021 Bitcraze AB, GPLv3
 */

#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

/* Include ARM CMSIS-DSP library */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "arm_math.h"
#pragma GCC diagnostic pop

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F (3.14159265358979323846f)
#endif

/* PI is already defined in arm_math.h */

#define DEG_TO_RAD (M_PI_F / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI_F)

#define GRAVITY_MAGNITUDE (9.81f)

#define SF_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define SF_MAX(a, b) (((a) > (b)) ? (a) : (b))

/* Small epsilon to prevent division by zero */
#define SF_EPS (1e-6f)

/*---------------------------------------------------------------------------
 * Assert macro - can be customized for your platform
 *---------------------------------------------------------------------------*/
#ifndef SF_ASSERT
#define SF_ASSERT(x) do { if (!(x)) { while(1); } } while(0)
#endif

/*---------------------------------------------------------------------------
 * Inline math helper functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Ensure matrix data is aligned on 4-byte boundaries
 */
static inline void sf_assert_aligned_4_bytes(const arm_matrix_instance_f32* matrix) {
    const uint32_t address = (uint32_t)(uintptr_t)matrix->pData;
    SF_ASSERT((address & 0x3) == 0);
}

/**
 * @brief Matrix transpose wrapper
 */
static inline void mat_trans(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_trans_f32(pSrc, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

/**
 * @brief Matrix inverse wrapper
 */
static inline void mat_inv(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_inverse_f32(pSrc, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

/**
 * @brief Matrix multiplication wrapper
 */
static inline void mat_mult(const arm_matrix_instance_f32* pSrcA, 
                            const arm_matrix_instance_f32* pSrcB, 
                            arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrcA);
    sf_assert_aligned_4_bytes(pSrcB);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_mult_f32(pSrcA, pSrcB, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

/**
 * @brief Matrix scale wrapper
 */
static inline void mat_scale(const arm_matrix_instance_f32* pSrc, 
                             float32_t scale,
                             arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_scale_f32(pSrc, scale, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

/**
 * @brief Square root using hardware FPU (Cortex-M7 VSQRT instruction)
 * @note On Cortex-M7 with FPU, sqrtf() compiles to the VSQRT.F32 instruction
 */
static inline float arm_sqrt(float32_t in) {
    if (in < 0.0f) {
        return 0.0f;
    }
    return sqrtf(in);
}

/* arm_cos_f32 and arm_sin_f32 are provided by arm_math.h / CMSIS-DSP */

/**
 * @brief Clamp positive (return 0 if negative)
 */
static inline float limPos(float in) {
    return (in < 0.0f) ? 0.0f : in;
}

/**
 * @brief Clamp value to range [min, max]
 */
static inline float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

