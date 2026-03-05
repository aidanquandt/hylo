#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

#define BACKOFF_CONFIG_DEFAULT                                                                     \
    {.min_delay_ms = 10, .max_delay_ms = 10000, .base_multiplier = 2, .use_jitter = true}

#define BACKOFF_CONFIG_FAST                                                                        \
    {.min_delay_ms = 1, .max_delay_ms = 1000, .base_multiplier = 2, .use_jitter = true}

#define BACKOFF_CONFIG_SLOW                                                                        \
    {.min_delay_ms = 100, .max_delay_ms = 60000, .base_multiplier = 2, .use_jitter = true}

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint32_t min_delay_ms;    ///< Minimum delay in milliseconds
    uint32_t max_delay_ms;    ///< Maximum delay in milliseconds (cap)
    uint32_t base_multiplier; ///< Multiplier for exponential growth (e.g., 2 for doubling)
    bool use_jitter;          ///< Add random jitter to prevent thundering herd
} backoff_config_t;

/*---------------------------------------------------------------------------
 * Public Functions
 *---------------------------------------------------------------------------*/
uint32_t backoff_calculate(const backoff_config_t* config, uint32_t attempt);
uint32_t backoff_calculate_simple(uint32_t attempt, uint32_t min_ms, uint32_t max_ms,
                                  bool use_jitter);
