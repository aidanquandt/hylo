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
#include "platform_gpio.h"

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
    for (uint8_t idx = 0U; idx < MAX_NUM_TASKS; idx++) 
    {
        cpu_usage[idx] = 0.0f;
    }
    
    uint32_t total_runtime = 0;

    // Note - uxTaskGetSystemState blocks scheduler - maybe should change later?
    UBaseType_t num_tasks = uxTaskGetSystemState(task_status_array, MAX_NUM_TASKS, &total_runtime);

    if ((num_tasks == 0U) || (total_runtime == 0U))
    {
        return;
    }

    for (UBaseType_t task_status_index = 0; task_status_index < num_tasks; ++task_status_index) 
    {
        TaskHandle_t task_handle = task_status_array[task_status_index].xHandle;

        for (uint8_t task_handle_index = 0; task_handle_index < MAX_NUM_TASKS; ++task_handle_index) {
            if ((task_handle_array[task_handle_index] != NULL) && 
                (task_handle == task_handle_array[task_handle_index])) 
            {
                float32_t percentage = (100.0f * (float32_t)task_status_array[task_status_index].ulRunTimeCounter) / (float32_t)total_runtime;
                cpu_usage[task_handle_index] = percentage;
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

STATIC void datalogger_init(void) 
{
    // init
}

STATIC void datalogger_process_1Hz(void)
{
    platform_gpio_toggle_led_green(); // todo - remove, helpful for debug.
    datalogger_monitor_rtos_usage();
}

STATIC void datalogger_process_100Hz(void)
{

}