#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/
typedef uint32_t platform_os_critical_state_t;
typedef void* platform_os_mutex_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void platform_os_init(void);
uint32_t platform_os_gettick(void);
void platform_os_delay_us_blocking(uint32_t delay_us);
void platform_os_delay_ms(uint32_t delay_ms);
platform_os_critical_state_t platform_os_critical_enter(void);
void platform_os_critical_exit(platform_os_critical_state_t state);
platform_os_mutex_t platform_os_mutex_create(void);
bool platform_os_mutex_take(platform_os_mutex_t mutex, uint32_t timeout_ms);
void platform_os_mutex_give(platform_os_mutex_t mutex);
