#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Get current timestamp from free-running hardware timer
 * @return Current timestamp value (handles wraparound)
 * @note Implementation-defined resolution, use platform_get_elapsed_us() for time measurements
 */
uint32_t platform_get_timestamp(void);

/**
 * @brief Get elapsed time in microseconds between two timestamps
 * @param start_timestamp Starting timestamp from platform_get_timestamp()
 * @param end_timestamp Ending timestamp from platform_get_timestamp()
 * @return Elapsed time in microseconds (handles counter wraparound)
 */
uint32_t platform_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp);