/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "backoff.h"
#include "common.h"
#include <stdlib.h>

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC uint32_t backoff_power(uint32_t base, uint32_t exponent);
STATIC uint32_t backoff_get_random_jitter(uint32_t delay_ms);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC uint32_t backoff_power(uint32_t base, uint32_t exponent)
{
    if (exponent == 0)
    {
        return 1;
    }

    uint32_t result = base;
    for (uint32_t i = 1; i < exponent; i++)
    {
        // Check for overflow before multiplication
        if (result > (UINT32_MAX / base))
        {
            return UINT32_MAX;
        }
        result *= base;
    }

    return result;
}

STATIC uint32_t backoff_get_random_jitter(uint32_t delay_ms)
{
    // Generate jitter: delay * (0.5 + 0.5 * rand)
    // Range: [delay * 0.5, delay * 1.0]
    uint32_t jitter_range  = delay_ms / 2; // 50% of delay
    uint32_t random_jitter = rand() % (jitter_range + 1);
    return (delay_ms / 2) + random_jitter;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

uint32_t backoff_calculate(const backoff_config_t* config, uint32_t attempt)
{
    if (config == NULL)
    {
        return 0;
    }

    // Start with minimum delay
    uint32_t delay_ms = config->min_delay_ms;

    // Calculate exponential delay: min_delay * base^attempt
    if (attempt > 0)
    {
        uint32_t multiplier = backoff_power(config->base_multiplier, attempt);

        // Check for overflow
        if (multiplier > (UINT32_MAX / config->min_delay_ms))
        {
            delay_ms = config->max_delay_ms;
        }
        else
        {
            delay_ms = config->min_delay_ms * multiplier;
        }
    }

    // Cap at maximum delay
    if (delay_ms > config->max_delay_ms)
    {
        delay_ms = config->max_delay_ms;
    }

    // Add jitter if enabled
    if (config->use_jitter)
    {
        delay_ms = backoff_get_random_jitter(delay_ms);
    }

    return delay_ms;
}

uint32_t backoff_calculate_simple(uint32_t attempt, uint32_t min_ms, uint32_t max_ms,
                                  bool use_jitter)
{
    backoff_config_t config = {.min_delay_ms    = min_ms,
                               .max_delay_ms    = max_ms,
                               .base_multiplier = 2,
                               .use_jitter      = use_jitter};

    return backoff_calculate(&config, attempt);
}
