/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STATIC static

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef float float32_t;
typedef double float64_t;

// todo - consider making this #define determined at compile time?
typedef enum
{
    NODE_TYPE_ANCHOR = 0U,
    NODE_TYPE_TAG,
    NUM_NODE_TYPES
} node_type_S;