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

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI_F / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI_F)
#endif

#ifndef GRAVITY_MAGNITUDE
#define GRAVITY_MAGNITUDE (9.81f)
#endif

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
static inline void sf_assert_aligned_4_bytes(const arm_matrix_instance_f32* matrix) {
    const uint32_t address = (uint32_t)(uintptr_t)matrix->pData;
    SF_ASSERT((address & 0x3) == 0);
}

static inline void mat_trans(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_trans_f32(pSrc, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

static inline void mat_inv(const arm_matrix_instance_f32* pSrc, arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_inverse_f32(pSrc, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

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

static inline void mat_scale(const arm_matrix_instance_f32* pSrc, 
                             float32_t scale,
                             arm_matrix_instance_f32* pDst) {
    sf_assert_aligned_4_bytes(pSrc);
    sf_assert_aligned_4_bytes(pDst);
    arm_status result = arm_mat_scale_f32(pSrc, scale, pDst);
    SF_ASSERT(ARM_MATH_SUCCESS == result);
    (void)result;
}

static inline float arm_sqrt(float32_t in) {
    if (in < 0.0f) {
        return 0.0f;
    }
    return sqrtf(in);
}

static inline float limPos(float in) {
    return (in < 0.0f) ? 0.0f : in;
}

static inline float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}
