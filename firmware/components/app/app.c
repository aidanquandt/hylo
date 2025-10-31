/*---------------------------------------------------------------------------
 * Includes :) :)
 *---------------------------------------------------------------------------*/
#include "app.h" // hello guys
#include "cmsis_os2.h"
#include "common.h"
#include "module.h"
#include "main.h"
#include "platform.h"
#include "platform_gpio.h"
#include "cmsis_os.h"
#include "task.h"
#include "datalogger.h"
#include "node.h"
#include "sensor_fusion.h"
#include "tdma.h"
#include "twr.h"

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
STATIC void module_task_1Hz(void *argument);
STATIC void module_task_10Hz(void *argument);
STATIC void module_task_100Hz(void *argument);
STATIC void module_task_1kHz(void *argument);
STATIC void app_initialize_modules(void);
STATIC void app_create_module_tasks(void);
STATIC void app_post_module_initialization(void);

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void module_task_1Hz(void *argument) 
{
    modules_E module = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        modules[module]->module_process_1Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_1HZ);
    }
}

STATIC void module_task_10Hz(void *argument) 
{
    modules_E module = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        modules[module]->module_process_10Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_10HZ);
    }
}

STATIC void module_task_100Hz(void *argument) 
{
    modules_E module = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        modules[module]->module_process_100Hz();
        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

STATIC void module_task_1kHz(void *argument) 
{
    modules_E module = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
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
    STATIC uint8_t task_num = 1U; // we will use task 0 as idle task in datalogger so just set this 1 as first task
    char task_name[16];

    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++) 
    {
        if (modules[module_idx]->module_process_1kHz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "TASK_%u", task_num);
            task_num++;

            xTaskCreate(
                module_task_1kHz, 
                task_name,
                TASK_STACK_MEDIUM,
                (void *)(uintptr_t)module_idx, 
                PRIORITY_1KHZ_TASK,
                NULL
            );
        }

        if (modules[module_idx]->module_process_100Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "TASK_%u", task_num);
            task_num++;

            xTaskCreate(
                module_task_100Hz, 
                task_name,
                TASK_STACK_MEDIUM,
                (void *)(uintptr_t)module_idx, 
                PRIORITY_100HZ_TASK,
                NULL
            );
        }

        if (modules[module_idx]->module_process_10Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "TASK_%u", task_num);
            task_num++;

            xTaskCreate(
                module_task_10Hz, 
                task_name,
                TASK_STACK_MEDIUM,
                (void *)(uintptr_t)module_idx, 
                PRIORITY_10HZ_TASK,
                NULL
            ); 
        }

        if (modules[module_idx]->module_process_1Hz != NULL)
        {
            snprintf(task_name, sizeof(task_name), "TASK_%u", task_num);
            task_num++;
            
            xTaskCreate(
                module_task_1Hz, 
                task_name,
                TASK_STACK_MEDIUM,
                (void *)(uintptr_t)module_idx, 
                PRIORITY_1HZ_TASK,
                NULL
            );
        }

    }
}

STATIC void app_post_module_initialization(void)
{
    datalogger_update_task_handles();
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void app_init(void) 
{
    // module initialization
    app_initialize_modules();
    app_create_module_tasks();

    app_post_module_initialization();

    osThreadExit();
}