#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

// Defines
#define STATIC static

// Macros
#define TOSTRING(x) #x
#define STRLEN_LITERAL(s) (sizeof(s) - 1)

// Time conversion macros
#define MS_TO_S(ms)     ((ms) / 1000U)
#define S_TO_MS(s)      ((s) * 1000U)
#define US_TO_MS(us)    ((us) / 1000U)
#define MS_TO_US(ms)    ((ms) * 1000U)
#define US_TO_S(us)     ((us) / 1000000U)
#define S_TO_US(s)      ((s) * 1000000U)

// Math macros
#define MIN(a, b)       (((a) < (b)) ? (a) : (b))
#define MAX(a, b)       (((a) > (b)) ? (a) : (b))
#define ABS(x)          (((x) < 0) ? -(x) : (x))
#define CLAMP(x, min, max) (MIN(MAX((x), (min)), (max)))

// todo - add tcm integration
// #define TCM_FUNCTION __attribute__((section(".itcm")))
// #define TCM_VARIABLE __attribute__((section(".dtcm")))

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;