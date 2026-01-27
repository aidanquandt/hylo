/*---------------------------------------------------------------------------
 * @file    twr_manager.h
 * @brief   Two-Way Ranging (TWR) Manager - Public API
 * @details High-level API for managing automatic ranging operations
 *---------------------------------------------------------------------------*/

#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
bool twr_manager_start(void);
void twr_manager_stop(void);
bool twr_manager_is_active(void);
uint32_t twr_manager_get_success_count(void);
uint32_t twr_manager_get_failure_count(void);
