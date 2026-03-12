#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/
typedef uint32_t os_driver_critical_state_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
void os_driver_init(void);
void os_driver_delay_us_blocking(uint32_t delay_us);
os_driver_critical_state_t os_driver_critical_enter(void);
void os_driver_critical_exit(os_driver_critical_state_t state);
