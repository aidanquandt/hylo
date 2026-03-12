#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
uint32_t timer_driver_get_timestamp(void);
uint32_t timer_driver_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp);
void timer_driver_on_overflow(void);
uint64_t timer_driver_get_time_us(void);
uint32_t timer_driver_get_time_ms(void);
