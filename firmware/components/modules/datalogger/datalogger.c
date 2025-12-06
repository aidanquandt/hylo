/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "datalogger.h"
#include "common.h"
#include "module.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"
#include "uart_manager.h"
#include "uart_cmd_router.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_NUM_TASKS 20U

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void datalogger_init(void);
STATIC void datalogger_process_1Hz(void);
STATIC void datalogger_process_100Hz(void);
STATIC bool datalogger_cmd_handler(const cmd_parsed_t *parsed);

extern const module_S datalogger_module;
const module_S datalogger_module = {
    .module_name = "datalogger",
    .module_init = datalogger_init,
    .module_process_1Hz = datalogger_process_1Hz,
    .module_process_100Hz = datalogger_process_100Hz,
    .module_cmd_handler = datalogger_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);
STATIC void datalogger_monitor_uart_health(void);

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/
STATIC float32_t cpu_usage[MAX_NUM_TASKS] = { 0 };
STATIC TaskStatus_t task_status_array[MAX_NUM_TASKS];
STATIC UBaseType_t num_tracked_tasks = 0;

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/

STATIC void datalogger_monitor_rtos_usage(void)
{
    STATIC uint32_t prev_total_runtime = 0U;
    STATIC uint32_t prev_task_runtime[MAX_NUM_TASKS] = { 0 };
    STATIC TaskHandle_t prev_task_handles[MAX_NUM_TASKS] = { NULL };

    uint32_t total_runtime = 0U;
    num_tracked_tasks = uxTaskGetSystemState(task_status_array, MAX_NUM_TASKS, &total_runtime);
    if ((num_tracked_tasks == 0U) || (total_runtime == 0U))
    {
        return;
    }

    // Handle counter wraparound for total runtime
    uint32_t total_runtime_change;
    if (total_runtime >= prev_total_runtime) {
        total_runtime_change = total_runtime - prev_total_runtime;
    } else {
        // Wraparound occurred
        total_runtime_change = (UINT32_MAX - prev_total_runtime) + total_runtime + 1;
    }
    
    // Skip if no time elapsed (same sample)
    if (total_runtime_change == 0U) {
        return;
    }

    // Process each task from current snapshot
    for (UBaseType_t task_index = 0U; task_index < num_tracked_tasks; ++task_index)
    {
        TaskHandle_t current_handle = task_status_array[task_index].xHandle;
        uint32_t current_runtime = task_status_array[task_index].ulRunTimeCounter;
        
        // Find this task in previous snapshot to calculate delta
        uint32_t previous_runtime = 0U;
        for (uint8_t prev_index = 0U; prev_index < MAX_NUM_TASKS; ++prev_index)
        {
            if (prev_task_handles[prev_index] == current_handle)
            {
                previous_runtime = prev_task_runtime[prev_index];
                break;
            }
        }

        // Handle counter wraparound for task runtime
        uint32_t runtime_change;
        if (current_runtime >= previous_runtime) {
            runtime_change = current_runtime - previous_runtime;
        } else {
            // Wraparound occurred
            runtime_change = (UINT32_MAX - previous_runtime) + current_runtime + 1;
        }
        
        float32_t usage_percent = 0.0f;
        if (total_runtime_change != 0U)
        {
            usage_percent = (100.0f * (float32_t)runtime_change) / (float32_t)total_runtime_change;
        }

        cpu_usage[task_index] = usage_percent;
        
        // Update previous values for next iteration
        prev_task_handles[task_index] = current_handle;
        prev_task_runtime[task_index] = current_runtime;
    }

    prev_total_runtime = total_runtime;
}


/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void datalogger_update_task_handles(void)
{
    // No longer needed - we get task info dynamically in monitor function
}

STATIC void datalogger_init(void) 
{
    // init
}

STATIC void datalogger_process_1Hz(void)
{
    datalogger_monitor_rtos_usage();
    datalogger_monitor_uart_health();
}

/*---------------------------------------------------------------------------
 * Command Handler
 *---------------------------------------------------------------------------*/
STATIC bool datalogger_cmd_handler(const cmd_parsed_t *parsed)
{
    switch (parsed->action) {
        case CMD_ACTION_GET:
            if (strcmp(parsed->target, "tasks") == 0) {
                uart_manager_print("\r\nTask List (CPU Usage):\r\n");
                uart_manager_print("%-20s %6s\r\n", "Task Name", "CPU %");
                uart_manager_print("-----------------------------\r\n");
                
                // Print all tasks using the snapshot from last monitoring cycle
                for (UBaseType_t i = 0; i < num_tracked_tasks; i++) {
                    const char *task_name = task_status_array[i].pcTaskName;
                    uart_manager_print("%-20s %5.2f%%\r\n", task_name, cpu_usage[i]);
                }
                
                uart_manager_print("\r\n");
                return true;
            }
            break;
            
        case CMD_ACTION_SET:
        case CMD_ACTION_REQ:
        case CMD_ACTION_UNKNOWN:
        default:
            break;
    }
    
    return false;
}

STATIC void datalogger_process_100Hz(void)
{

}

/**
 * @brief Monitor UART queue health and report anomalies
 * @details Tracks queue depth and dropped messages to detect logging storms
 */
STATIC void datalogger_monitor_uart_health(void)
{
    uint32_t queue_count = uart_manager_get_queue_count();
    STATIC uint32_t prev_dropped = 0U;
    uint32_t dropped = uart_manager_get_dropped_count();
    
    // Check if messages were dropped since last check
    if (dropped > prev_dropped) {
        uint32_t new_drops = dropped - prev_dropped;
        // Note: This will attempt to print, might drop if queue still full
        // In production, could set an error LED instead
        uart_manager_print("[UART] %u messages dropped!\n", (unsigned int)new_drops);
        prev_dropped = dropped;
    }
    
    // Warn if queue is filling up (>75% full = 24/32 messages)
    if (queue_count > 24U) {
        uart_manager_print("[UART] Queue high: %u/32\n", (unsigned int)queue_count);
    }
}