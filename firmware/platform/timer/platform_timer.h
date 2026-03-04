#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/** Raw timer tick (for elapsed / stopwatch). Same source as run-time stats (TIM5). */
uint32_t platform_get_timestamp(void);

/** Elapsed microseconds between two raw timestamps. */
uint32_t platform_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp);

/**
 * Called from HAL_TIM_PeriodElapsedCallback when TIM2 overflows (every 2^32 µs).
 * Do not call from application code.
 */
void platform_timer_on_overflow(void);

/**
 * Current time in microseconds since boot (64-bit, overflow-safe).
 * Uses TIM2 + overflow interrupt; use for sensor fusion / event timestamps.
 */
uint64_t platform_get_time_us(void);

/** Current time in milliseconds (platform_get_time_us() / 1000). Truncates to 32-bit for timestamp_ms API. */
uint32_t platform_get_time_ms(void);