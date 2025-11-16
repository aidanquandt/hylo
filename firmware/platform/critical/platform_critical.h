/*---------------------------------------------------------------------------
 * @file    platform_critical.h
 * @brief   Platform abstraction for critical sections and mutual exclusion
 * @note    Wraps RTOS-specific primitives to allow port layers to be RTOS-agnostic
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/** Type for storing critical section state (opaque to callers) */
typedef uint32_t platform_critical_state_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Enter a critical section (disable interrupts/scheduler)
 * @return State value to be passed to platform_critical_exit()
 * @note Must be paired with platform_critical_exit()
 * @note Not reentrant - do not nest critical sections from same context
 */
platform_critical_state_t platform_critical_enter(void);

/**
 * @brief Exit a critical section (restore interrupts/scheduler)
 * @param state State value returned from platform_critical_enter()
 * @note Must be called with the state from the matching enter call
 */
void platform_critical_exit(platform_critical_state_t state);
