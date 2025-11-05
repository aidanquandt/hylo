#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Delay execution for specified milliseconds (RTOS-friendly)
 * @param ms Delay time in milliseconds
 */
void platform_delay_ms(uint32_t ms);

/**
 * @brief Delay execution for specified microseconds (blocking)
 * @param us Delay time in microseconds
 * @note Uses DWT cycle counter for precise timing, blocks CPU
 */
void platform_delay_us(uint32_t us);