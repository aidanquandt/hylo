/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "common.h"
#include "datalogger.h"
#include "error_handler.h"
#include "main.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "sensor_fusion.h"
#include "task.h"
#include "uart_cmd_router.h"
#include "uart_manager.h"
#include "uwb.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TASK_RATE_1KHZ pdMS_TO_TICKS(1)
#define TASK_RATE_100HZ pdMS_TO_TICKS(10)
#define TASK_RATE_10HZ pdMS_TO_TICKS(100)
#define TASK_RATE_1HZ pdMS_TO_TICKS(1000)

#define PRIORITY_1KHZ_TASK 4
#define PRIORITY_100HZ_TASK 3
#define PRIORITY_10HZ_TASK 2
#define PRIORITY_1HZ_TASK 1

#define TASK_STACK_SMALL 256
#define TASK_STACK_MEDIUM 512
#define TASK_STACK_LARGE 1024

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void module_task_1Hz(void* argument);
STATIC void module_task_10Hz(void* argument);
STATIC void module_task_100Hz(void* argument);
STATIC void module_task_1kHz(void* argument);
STATIC void app_initialize_modules(void);
STATIC void app_create_module_tasks(void);
STATIC void app_post_module_initialization(void);

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void module_task_1Hz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        modules[module]->module_process_1Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_1HZ);
    }
}

STATIC void module_task_10Hz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        modules[module]->module_process_10Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_10HZ);
    }
}

STATIC void module_task_100Hz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        modules[module]->module_process_100Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

STATIC void module_task_1kHz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        modules[module]->module_process_1kHz();
        vTaskDelayUntil(&lastWake, TASK_RATE_1KHZ);
    }
}

STATIC void app_initialize_modules(void)
{
    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++)
    {
        if (modules[module_idx]->module_init != NULL)
        {
            modules[module_idx]->module_init();
        }
    }
}

STATIC void app_create_module_tasks(void)
{
    char task_name[32]; // Match configMAX_TASK_NAME_LEN
    BaseType_t result;

    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++)
    {
        if (modules[module_idx]->module_create_task != NULL)
        {
            modules[module_idx]->module_create_task();
        }
    }

    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++)
    {
        if (modules[module_idx]->module_process_1kHz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "%s_1k",
                     modules[module_idx]->module_name ? modules[module_idx]->module_name : "MOD");

            result = xTaskCreate(module_task_1kHz, task_name, TASK_STACK_MEDIUM,
                                 (void*)(uintptr_t)module_idx, PRIORITY_1KHZ_TASK, NULL);
            if (result != pdPASS)
            {
                error_handler_fatal("app", "Failed to create task '%s' (1kHz)", task_name);
            }
        }

        if (modules[module_idx]->module_process_100Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "%s_100",
                     modules[module_idx]->module_name ? modules[module_idx]->module_name : "MOD");

            result = xTaskCreate(module_task_100Hz, task_name, TASK_STACK_MEDIUM,
                                 (void*)(uintptr_t)module_idx, PRIORITY_100HZ_TASK, NULL);
            if (result != pdPASS)
            {
                error_handler_fatal("app", "Failed to create task '%s' (100Hz)", task_name);
            }
        }

        if (modules[module_idx]->module_process_10Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "%s_10",
                     modules[module_idx]->module_name ? modules[module_idx]->module_name : "MOD");

            result = xTaskCreate(module_task_10Hz, task_name, TASK_STACK_MEDIUM,
                                 (void*)(uintptr_t)module_idx, PRIORITY_10HZ_TASK, NULL);
            if (result != pdPASS)
            {
                error_handler_fatal("app", "Failed to create task '%s' (10Hz)", task_name);
            }
        }

        if (modules[module_idx]->module_process_1Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "%s_1",
                     modules[module_idx]->module_name ? modules[module_idx]->module_name : "MOD");

            result = xTaskCreate(module_task_1Hz, task_name, TASK_STACK_MEDIUM,
                                 (void*)(uintptr_t)module_idx, PRIORITY_1HZ_TASK, NULL);
            if (result != pdPASS)
            {
                error_handler_fatal("app", "Failed to create task '%s' (1Hz)", task_name);
            }
        }
    }
}

STATIC void app_post_module_initialization(void)
{
    uart_cmd_router_init();
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void app_init(void)
{
    app_initialize_modules();
    app_create_module_tasks();
    app_post_module_initialization();
    vTaskDelete(NULL); // Delete init task - scheduler continues with created tasks
}