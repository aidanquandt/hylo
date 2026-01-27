/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "datalogger.h"
#include "FreeRTOS.h"
#include "app.h"
#include "common.h"
#include "error_handler.h"
#include "main.h"
#include "module.h"
#include "task.h"
#include "uart_manager.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_NUM_TASKS 20U

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void datalogger_process_1Hz(void);

extern const module_S datalogger_module;
const module_S datalogger_module = {
    .module_name        = "datalogger",
    .module_process_1Hz = datalogger_process_1Hz,
};

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);
STATIC void datalogger_monitor_uart_health(void);

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/
STATIC float32_t cpu_usage[MAX_NUM_TASKS] = {0};
STATIC TaskStatus_t task_status_array[MAX_NUM_TASKS];
STATIC UBaseType_t num_tracked_tasks = 0;

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void)
{
    STATIC uint32_t prev_total_runtime                   = 0U;
    STATIC uint32_t prev_task_runtime[MAX_NUM_TASKS]     = {0};
    STATIC TaskHandle_t prev_task_handles[MAX_NUM_TASKS] = {NULL};

    uint32_t total_runtime = 0U;
    num_tracked_tasks      = uxTaskGetSystemState(task_status_array, MAX_NUM_TASKS, &total_runtime);
    if ((num_tracked_tasks == 0U) || (total_runtime == 0U))
    {
        return;
    }

    uint32_t total_runtime_change;
    if (total_runtime >= prev_total_runtime)
    {
        total_runtime_change = total_runtime - prev_total_runtime;
    }
    else
    {
        total_runtime_change = (UINT32_MAX - prev_total_runtime) + total_runtime + 1;
    }

    if (total_runtime_change == 0U)
    {
        return;
    }

    for (UBaseType_t task_index = 0U; task_index < num_tracked_tasks; ++task_index)
    {
        TaskHandle_t current_handle = task_status_array[task_index].xHandle;
        uint32_t current_runtime    = task_status_array[task_index].ulRunTimeCounter;

        uint32_t previous_runtime = 0U;
        for (uint8_t prev_index = 0U; prev_index < MAX_NUM_TASKS; ++prev_index)
        {
            if (prev_task_handles[prev_index] == current_handle)
            {
                previous_runtime = prev_task_runtime[prev_index];
                break;
            }
        }

        uint32_t runtime_change;
        if (current_runtime >= previous_runtime)
        {
            runtime_change = current_runtime - previous_runtime;
        }
        else
        {
            runtime_change = (UINT32_MAX - previous_runtime) + current_runtime + 1;
        }

        float32_t usage_percent = 0.0f;
        if (total_runtime_change != 0U)
        {
            usage_percent = (100.0f * (float32_t)runtime_change) / (float32_t)total_runtime_change;
        }

        cpu_usage[task_index] = usage_percent;

        prev_task_handles[task_index] = current_handle;
        prev_task_runtime[task_index] = current_runtime;
    }

    prev_total_runtime = total_runtime;
}

/*---------------------------------------------------------------------------
 * Module Implementation
 *---------------------------------------------------------------------------*/
STATIC void datalogger_process_1Hz(void)
{
    datalogger_monitor_rtos_usage();
    datalogger_monitor_uart_health();
}

/*---------------------------------------------------------------------------
 * Public Function Implementation
 *---------------------------------------------------------------------------*/

uint32_t datalogger_get_task_usage(task_cpu_info_t* tasks, uint32_t max_tasks)
{
    if (tasks == NULL || max_tasks == 0)
    {
        return 0;
    }

    uint32_t count = (num_tracked_tasks < max_tasks) ? num_tracked_tasks : max_tasks;
    for (uint32_t i = 0; i < count; i++)
    {
        tasks[i].task_name   = task_status_array[i].pcTaskName;
        tasks[i].cpu_percent = cpu_usage[i];
    }

    return count;
}

STATIC void datalogger_monitor_uart_health(void)
{
    uint32_t queue_count         = uart_manager_get_queue_count();
    STATIC uint32_t prev_dropped = 0U;
    uint32_t dropped             = uart_manager_get_dropped_count();

    if (dropped > prev_dropped)
    {
        uint32_t new_drops = dropped - prev_dropped;
        error_handler_log(ERROR_SEVERITY_ERROR, "datalogger", "[UART] %u messages dropped!\n",
                          (unsigned int)new_drops);
        prev_dropped = dropped;
    }

    if (queue_count > 24U)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "datalogger", "[UART] Queue high: %u/32\n",
                          (unsigned int)queue_count);
    }
}