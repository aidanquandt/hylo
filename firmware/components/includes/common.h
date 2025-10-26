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

// todo - add tcm integration
// #define TCM_FUNCTION __attribute__((section(".itcm")))
// #define TCM_VARIABLE __attribute__((section(".dtcm")))

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;