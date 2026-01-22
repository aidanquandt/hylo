#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/
typedef uint32_t platform_os_critical_state_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void platform_os_init(void);
void platform_os_delay_us_blocking(uint32_t delay_us);
bool platform_os_is_in_isr(void);
platform_os_critical_state_t platform_os_critical_enter(void);
void platform_os_critical_exit(platform_os_critical_state_t state);
