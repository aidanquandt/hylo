#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "common.h"
#include "task.h"


/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Register current task for watchdog monitoring
 *
 * Call this once from each task that should be monitored. The watchdog will
 * expect periodic heartbeats. If a task stops calling watchdog_heartbeat(),
 * the system will reset via hardware watchdog.
 *
 * @param expected_period_ms Expected maximum time between heartbeats
 */
void watchdog_register_task(uint32_t expected_period_ms);

/**
 * @brief Send heartbeat from current task
 *
 * Call this periodically from your task loop to indicate the task is healthy.
 * Must call watchdog_register_task() first.
 */
void watchdog_heartbeat(void);
