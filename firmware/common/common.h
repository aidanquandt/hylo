#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

// Defines
#define STATIC static

// Macros
#define TOSTRING(x) #x
#define STRLEN_LITERAL(s) (sizeof(s) - 1)

// Time conversion macros
#define MS_TO_S(ms) ((ms) / 1000U)
#define S_TO_MS(s) ((s) * 1000U)
#define US_TO_MS(us) ((us) / 1000U)
#define MS_TO_US(ms) ((ms) * 1000U)
#define US_TO_S(us) ((us) / 1000000U)
#define S_TO_US(s) ((s) * 1000000U)

// Frequency tick conversion macros
#define MS_TO_1HZ_TICKS(ms) ((ms) / 1000U) // Convert ms to 1Hz ticks (1000ms period)
#define MS_TO_10HZ_TICKS(ms) ((ms) / 100U) // Convert ms to 10Hz ticks (100ms period)
#define MS_TO_100HZ_TICKS(ms) ((ms) / 10U) // Convert ms to 100Hz ticks (10ms period)
#define MS_TO_1KHZ_TICKS(ms) (ms)          // Convert ms to 1kHz ticks (1ms period)
#define S_TO_1HZ_TICKS(s) (s)              // Convert s to 1Hz ticks (1s period)
#define S_TO_10HZ_TICKS(s) ((s) * 10U)     // Convert s to 10Hz ticks (100ms period)
#define S_TO_100HZ_TICKS(s) ((s) * 100U)   // Convert s to 100Hz ticks (10ms period)
#define S_TO_1KHZ_TICKS(s) ((s) * 1000U)   // Convert s to 1kHz ticks (1ms period)

// Math macros
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define ABS(x) (((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) (MIN(MAX((x), (min)), (max)))

/*---------------------------------------------------------------------------
 * Common Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Generic 3D vector/position type
 * @note Used for positions, velocities, accelerations, etc.
 */
typedef struct
{
    float x;
    float y;
    float z;
} vec3_t;

// todo - add tcm integration
// #define TCM_FUNCTION __attribute__((section(".itcm")))
// #define TCM_VARIABLE __attribute__((section(".dtcm")))

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;