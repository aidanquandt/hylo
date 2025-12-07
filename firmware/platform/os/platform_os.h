#pragma once

/*---------------------------------------------------------------------------
 * @file    platform_os.h
 * @brief   Operating system abstraction layer
 *          Provides RTOS-independent delay and synchronization primitives
 *---------------------------------------------------------------------------*/

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
 * Public function prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize platform OS layer
 * @note Must be called once at startup before using delay functions
 */
void platform_os_init(void);

/**
 * @brief Get current system tick count in milliseconds
 * @return Current tick count
 */
uint32_t platform_os_gettick(void);

/**
 * @brief Blocking delay in microseconds
 * @param delay_us Delay time in microseconds (max 10000 = 10ms)
 * @note Uses busy-wait loop with scheduler suspended for accuracy
 * @note Interrupts remain enabled, SysTick continues running
 * @note NOT safe to call from ISR context due to vTaskSuspendAll()
 * @note Delays > 10ms automatically convert to yielding ms delay
 */
void platform_os_delay_us_blocking(uint32_t delay_us);

/**
 * @brief Delay in milliseconds (yields to scheduler)
 * @param delay_ms Delay time in milliseconds
 * @note Yields to RTOS scheduler, allowing other tasks to run
 * @note Safe to call from task context only (not ISR)
 */
void platform_os_delay_ms(uint32_t delay_ms);

/**
 * @brief Enter critical section (disable interrupts)
 * @return Previous interrupt state
 */
platform_os_critical_state_t platform_os_critical_enter(void);

/**
 * @brief Exit critical section (restore interrupts)
 * @param state Previous interrupt state from platform_os_critical_enter()
 */
void platform_os_critical_exit(platform_os_critical_state_t state);

/**
 * @brief Create a mutex for synchronization
 * @return Mutex handle, or NULL on failure
 * @note Must be called before scheduler starts
 */
platform_os_mutex_t platform_os_mutex_create(void);

/**
 * @brief Acquire mutex with timeout
 * @param mutex Mutex handle
 * @param timeout_ms Timeout in milliseconds (0 = no wait, UINT32_MAX = wait forever)
 * @return true if mutex acquired, false on timeout
 * @note Safe to call from task context only (not ISR)
 */
bool platform_os_mutex_take(platform_os_mutex_t mutex, uint32_t timeout_ms);

/**
 * @brief Release mutex
 * @param mutex Mutex handle
 * @note Safe to call from task context only (not ISR)
 */
void platform_os_mutex_give(platform_os_mutex_t mutex);
