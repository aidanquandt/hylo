#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Platform Modules
 * 
 * The platform layer provides hardware abstraction for:
 *   - GPIO: Pin control (platform_gpio.h)
 *   - SPI: Serial communication (platform_spi.h)  
 *   - Timer: Delays and timing (platform_timer.h)
 *   - Critical: Thread-safe sections (platform_critical.h)
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/
uint32_t platform_gettick(void);
void platform_delay_ms(uint32_t delay_ms);
