#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Exponential backoff configuration
 */
typedef struct
{
    uint32_t min_delay_ms;    ///< Minimum delay in milliseconds
    uint32_t max_delay_ms;    ///< Maximum delay in milliseconds (cap)
    uint32_t base_multiplier; ///< Multiplier for exponential growth (e.g., 2 for doubling)
    bool use_jitter;          ///< Add random jitter to prevent thundering herd
} backoff_config_t;

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

/**
 * @brief Default backoff configuration: 10ms to 10s, doubling, with jitter
 */
#define BACKOFF_CONFIG_DEFAULT                                                                     \
    {.min_delay_ms = 10, .max_delay_ms = 10000, .base_multiplier = 2, .use_jitter = true}

/**
 * @brief Fast backoff: 1ms to 1s, doubling, with jitter
 */
#define BACKOFF_CONFIG_FAST                                                                        \
    {.min_delay_ms = 1, .max_delay_ms = 1000, .base_multiplier = 2, .use_jitter = true}

/**
 * @brief Slow backoff: 100ms to 60s, doubling, with jitter
 */
#define BACKOFF_CONFIG_SLOW                                                                        \
    {.min_delay_ms = 100, .max_delay_ms = 60000, .base_multiplier = 2, .use_jitter = true}

/*---------------------------------------------------------------------------
 * Public Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Calculate exponential backoff delay
 *
 * Calculates delay using: min(max_delay, min_delay * base^attempt)
 * Optionally adds jitter: delay * (0.5 to 1.0) random factor
 *
 * @param config Backoff configuration
 * @param attempt Attempt number (0 = first attempt, 1 = first retry, etc.)
 * @return Delay in milliseconds
 *
 * @note This function is stateless and reentrant
 *
 * Example:
 *   backoff_config_t cfg = BACKOFF_CONFIG_DEFAULT;
 *   uint32_t delay = backoff_calculate(&cfg, retry_count);
 */
uint32_t backoff_calculate(const backoff_config_t* config, uint32_t attempt);

/**
 * @brief Calculate exponential backoff with custom range
 *
 * Convenience function for one-off calculations without creating config struct
 *
 * @param attempt Attempt number (0-based)
 * @param min_ms Minimum delay in milliseconds
 * @param max_ms Maximum delay in milliseconds
 * @param use_jitter Whether to add random jitter
 * @return Delay in milliseconds
 */
uint32_t backoff_calculate_simple(uint32_t attempt, uint32_t min_ms, uint32_t max_ms,
                                  bool use_jitter);
