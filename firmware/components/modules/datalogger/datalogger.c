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

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_NUM_TASKS 10U

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void datalogger_init(void);
STATIC void datalogger_process_100Hz(void);

extern const module_S datalogger_module;
const module_S datalogger_module = {
    .module_init = datalogger_init,
    .module_process_100Hz = datalogger_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/
STATIC float32_t cpu_usage[MAX_NUM_TASKS] = { 0.0f }; //combine cpu usage and task handle array
STATIC TaskStatus_t task_status_array[MAX_NUM_TASKS];
STATIC TaskHandle_t task_handle_array[MAX_NUM_TASKS];

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/

STATIC void datalogger_monitor_rtos_usage(void)
{
    for (uint8_t i = 0; i < MAX_NUM_TASKS; ++i) cpu_usage[i] = 0.0f;

    uint32_t total_runtime = 0;

    UBaseType_t num_tasks = uxTaskGetSystemState(task_status_array, MAX_NUM_TASKS, &total_runtime);

    if (num_tasks == 0 || total_runtime == 0)
    {
        return;
    }

    for (UBaseType_t tsi = 0; tsi < num_tasks; ++tsi) {
        TaskHandle_t h = task_status_array[tsi].xHandle;

        for (uint8_t idx = 0; idx < MAX_NUM_TASKS; ++idx) {
            if (task_handle_array[idx] != NULL && h == task_handle_array[idx]) {
                float32_t percentage = (100.0f * (float32_t)task_status_array[tsi].ulRunTimeCounter) / (float32_t)total_runtime;
                cpu_usage[idx] = percentage;
                break;
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void datalogger_update_task_handles(void)
{
    for (uint8_t i = 0; i < MAX_NUM_TASKS; ++i)
    {
        task_handle_array[i] = NULL;
    } 

    task_handle_array[0] = xTaskGetHandle("IDLE");
    task_handle_array[1] = xTaskGetHandle("Tmr Svc");
    if (task_handle_array[1] == NULL) task_handle_array[1] = xTaskGetHandle("TmrSvc");
    if (task_handle_array[1] == NULL) task_handle_array[1] = xTaskGetHandle("Timer");

    for (uint8_t k = 2, num = 1; k < MAX_NUM_TASKS; ++k, ++num) {
        char name[16];
        snprintf(name, sizeof(name), "TASK_%u", (unsigned)num);
        task_handle_array[k] = xTaskGetHandle(name);
    }
}

STATIC void datalogger_init(void) {
    // init
}

STATIC void datalogger_process_100Hz(void) {

    STATIC uint32_t monitor_rtos_usage_count = 0U;
    if ((monitor_rtos_usage_count % 20U) == 0U)
    {
        // only run this at 5 Hz as it is quite expensive. todo - optimize datalogger_monitor_rtos_usage
        // todo - make counter api
        datalogger_monitor_rtos_usage();
    }
    monitor_rtos_usage_count++;

    // runtime_stats_bufffer updated at this point can send here
}