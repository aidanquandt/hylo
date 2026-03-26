/**
 * @file arm_math_shim.h
 * @brief Desktop replacement for ARM CMSIS-DSP arm_math.h
 *
 * Provides the subset of CMSIS-DSP types and functions used by kalman_core.c
 * so that it can be compiled as a shared library on x86/ARM64 desktop
 * platforms for offline analysis via Python ctypes.
 */
#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef float float32_t;

typedef enum {
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_NANINF = -4,
    ARM_MATH_SINGULAR = -5,
    ARM_MATH_TEST_FAILURE = -6
} arm_status;

typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float32_t *pData;
} arm_matrix_instance_f32;

/*---------------------------------------------------------------------------
 * Matrix operations
 *---------------------------------------------------------------------------*/

static inline arm_status arm_mat_trans_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pDst)
{
    for (uint16_t r = 0; r < pSrc->numRows; r++) {
        for (uint16_t c = 0; c < pSrc->numCols; c++) {
            pDst->pData[c * pSrc->numRows + r] =
                pSrc->pData[r * pSrc->numCols + c];
        }
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst)
{
    uint16_t m = pSrcA->numRows;
    uint16_t n = pSrcB->numCols;
    uint16_t k = pSrcA->numCols;

    for (uint16_t i = 0; i < m; i++) {
        for (uint16_t j = 0; j < n; j++) {
            float32_t sum = 0.0f;
            for (uint16_t p = 0; p < k; p++) {
                sum += pSrcA->pData[i * k + p] * pSrcB->pData[p * n + j];
            }
            pDst->pData[i * n + j] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_scale_f32(
    const arm_matrix_instance_f32 *pSrc,
    float32_t scale,
    arm_matrix_instance_f32 *pDst)
{
    uint32_t total = (uint32_t)pSrc->numRows * pSrc->numCols;
    for (uint32_t i = 0; i < total; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
    }
    return ARM_MATH_SUCCESS;
}

/**
 * @brief Matrix inverse via Gauss-Jordan elimination (in-place on copy).
 *        Only for small square matrices (9x9 in our use case).
 */
static inline arm_status arm_mat_inverse_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pDst)
{
    uint16_t n = pSrc->numRows;
    if (n != pSrc->numCols) return ARM_MATH_SIZE_MISMATCH;

    /* Work on a copy so we don't destroy pSrc */
    float32_t tmp[n * n];
    memcpy(tmp, pSrc->pData, n * n * sizeof(float32_t));

    /* Initialize pDst as identity */
    memset(pDst->pData, 0, n * n * sizeof(float32_t));
    for (uint16_t i = 0; i < n; i++) {
        pDst->pData[i * n + i] = 1.0f;
    }

    for (uint16_t col = 0; col < n; col++) {
        /* Partial pivoting */
        float32_t maxVal = fabsf(tmp[col * n + col]);
        uint16_t maxRow = col;
        for (uint16_t row = col + 1; row < n; row++) {
            float32_t v = fabsf(tmp[row * n + col]);
            if (v > maxVal) {
                maxVal = v;
                maxRow = row;
            }
        }

        if (maxVal < 1e-12f) return ARM_MATH_SINGULAR;

        /* Swap rows */
        if (maxRow != col) {
            for (uint16_t j = 0; j < n; j++) {
                float32_t t = tmp[col * n + j];
                tmp[col * n + j] = tmp[maxRow * n + j];
                tmp[maxRow * n + j] = t;

                t = pDst->pData[col * n + j];
                pDst->pData[col * n + j] = pDst->pData[maxRow * n + j];
                pDst->pData[maxRow * n + j] = t;
            }
        }

        /* Scale pivot row */
        float32_t pivot = tmp[col * n + col];
        for (uint16_t j = 0; j < n; j++) {
            tmp[col * n + j] /= pivot;
            pDst->pData[col * n + j] /= pivot;
        }

        /* Eliminate other rows */
        for (uint16_t row = 0; row < n; row++) {
            if (row == col) continue;
            float32_t factor = tmp[row * n + col];
            for (uint16_t j = 0; j < n; j++) {
                tmp[row * n + j] -= factor * tmp[col * n + j];
                pDst->pData[row * n + j] -= factor * pDst->pData[col * n + j];
            }
        }
    }

    return ARM_MATH_SUCCESS;
}

/*---------------------------------------------------------------------------
 * Fast math functions (just use standard libm)
 *---------------------------------------------------------------------------*/

static inline float32_t arm_cos_f32(float32_t x) { return cosf(x); }
static inline float32_t arm_sin_f32(float32_t x) { return sinf(x); }
