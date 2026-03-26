/**
 * @file analysis_common.h
 * @brief Minimal desktop shim for common.h
 *
 * Replaces the firmware common.h for desktop compilation.
 * Only defines what kalman_core.c actually needs.
 */
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STATIC static

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_F
#define M_PI_F (3.14159265358979323846f)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI_F / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI_F)
#endif

#ifndef GRAVITY_MAGNITUDE
#define GRAVITY_MAGNITUDE (9.81f)
#endif

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define ABS(x) (((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) (MIN(MAX((x), (min)), (max)))

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;
