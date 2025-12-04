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

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_NUM_TASKS 10U

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void datalogger_init(void);
STATIC void datalogger_process_1Hz(void);
STATIC void datalogger_process_100Hz(void);

extern const module_S datalogger_module;
const module_S datalogger_module = {
    .module_init = datalogger_init,
    .module_process_1Hz = datalogger_process_1Hz,
    .module_process_100Hz = datalogger_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);
STATIC void datalogger_monitor_uart_health(void);

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/
STATIC float32_t cpu_usage[MAX_NUM_TASKS] = { 0 }; //combine cpu usage and task handle array
STATIC float32_t idle_task_cpu_usage = 0.0f;
STATIC TaskStatus_t task_status_array[MAX_NUM_TASKS];
STATIC TaskHandle_t task_handle_array[MAX_NUM_TASKS];

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/

STATIC void datalogger_monitor_rtos_usage(void)
{
    STATIC uint32_t prev_total_runtime = 0U;
    STATIC uint32_t prev_task_runtime[MAX_NUM_TASKS] = { 0 };

    uint32_t total_runtime = 0U;
    UBaseType_t num_active_tasks = uxTaskGetSystemState(task_status_array, MAX_NUM_TASKS, &total_runtime);
    if ((num_active_tasks == 0U) || (total_runtime == 0U) || (total_runtime == prev_total_runtime))
    {
        return;
    }

    uint32_t total_runtime_change = total_runtime - prev_total_runtime;

    for (UBaseType_t task_index = 0U; task_index < num_active_tasks; ++task_index)
    {
        TaskHandle_t current_handle = task_status_array[task_index].xHandle;

        for (uint8_t handle_index = 0U; handle_index < MAX_NUM_TASKS; ++handle_index)
        {
            if (task_handle_array[handle_index] == current_handle)
            {
                uint32_t current_runtime = task_status_array[task_index].ulRunTimeCounter;
                uint32_t previous_runtime = prev_task_runtime[handle_index];
                uint32_t runtime_change = current_runtime - previous_runtime;

                float32_t usage_percent = 0.0f;
                if (total_runtime_change != 0U)
                {
                    usage_percent = (100.0f * (float32_t)runtime_change) / (float32_t)total_runtime_change;
                }

                if (handle_index == 0U)
                {
                    idle_task_cpu_usage = usage_percent;
                }
                
                cpu_usage[handle_index] = usage_percent;
                prev_task_runtime[handle_index] = current_runtime;
                break;
            }
        }
    }

    prev_total_runtime = total_runtime;
}


/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void datalogger_update_task_handles(void)
{
    for (uint8_t handle_index = 0; handle_index < MAX_NUM_TASKS; handle_index++)
    {
        task_handle_array[handle_index] = NULL;
    } 

    task_handle_array[0] = xTaskGetHandle("IDLE");
    task_handle_array[1] = xTaskGetHandle("Tmr Svc");

    for (uint8_t task_slot = 2U; task_slot < MAX_NUM_TASKS; ++task_slot)
    {
        char task_name[16];
        // Task numbers start at 1, corresponding to slot index 2 → TASK_1
        unsigned task_number = (unsigned)(task_slot - 1U);
        snprintf(task_name, sizeof(task_name), "TASK_%u", task_number);

        task_handle_array[task_slot] = xTaskGetHandle(task_name);
    }
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