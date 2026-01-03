#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Task CPU usage information
 */
typedef struct
{
    const char* task_name; ///< Task name
    float cpu_percent;     ///< CPU usage percentage
} task_cpu_info_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Get CPU usage for all tasks
 * @param tasks Output: Array of task CPU info
 * @param max_tasks Maximum number of tasks to retrieve
 * @return Number of tasks retrieved
 */
uint32_t datalogger_get_task_usage(task_cpu_info_t* tasks, uint32_t max_tasks);