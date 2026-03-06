/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "common.h"
#include <stdio.h>
#include "datalogger.h"
#include "error_handler.h"
#include "feature_config.h"
#include "main.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "platform_system.h"
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
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC volatile uint32_t deadline_miss_count[NUM_MODULES] = {0};
STATIC volatile uint32_t worst_latency_ticks[NUM_MODULES] = {0};
STATIC volatile uint32_t module_period_ms[NUM_MODULES] = {0};

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
    module_period_ms[module] = 1000;
    for (;;)
    {
        modules[module]->module_process_1Hz();

        // Detect deadline miss (non-blocking)
        TickType_t now          = xTaskGetTickCount();
        TickType_t expectedWake = lastWake + TASK_RATE_1HZ;
        if (now > expectedWake)  // Only count as miss if actually late
        {
            TickType_t latency = now - expectedWake;
            deadline_miss_count[module]++;
            if (latency > worst_latency_ticks[module])
            {
                worst_latency_ticks[module] = latency;
            }
        }

        vTaskDelayUntil(&lastWake, TASK_RATE_1HZ);
    }
}

STATIC void module_task_10Hz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    module_period_ms[module] = 100;
    for (;;)
    {
        modules[module]->module_process_10Hz();

        // Detect deadline miss (non-blocking)
        TickType_t now          = xTaskGetTickCount();
        TickType_t expectedWake = lastWake + TASK_RATE_10HZ;
        if (now > expectedWake)  // Only count as miss if actually late
        {
            TickType_t latency = now - expectedWake;
            deadline_miss_count[module]++;
            if (latency > worst_latency_ticks[module])
            {
                worst_latency_ticks[module] = latency;
            }
        }

        vTaskDelayUntil(&lastWake, TASK_RATE_10HZ);
    }
}

STATIC void module_task_100Hz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    module_period_ms[module] = 10;
    for (;;)
    {
        modules[module]->module_process_100Hz();

        // Detect deadline miss (non-blocking)
        TickType_t now          = xTaskGetTickCount();
        TickType_t expectedWake = lastWake + TASK_RATE_100HZ;
        if (now > expectedWake)  // Only count as miss if actually late
        {
            TickType_t latency = now - expectedWake;
            deadline_miss_count[module]++;
            if (latency > worst_latency_ticks[module])
            {
                worst_latency_ticks[module] = latency;
            }
        }

        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

STATIC void module_task_1kHz(void* argument)
{
    modules_E module    = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    module_period_ms[module] = 1;
    for (;;)
    {
        modules[module]->module_process_1kHz();

        // Detect deadline miss (non-blocking)
        TickType_t now          = xTaskGetTickCount();
        TickType_t expectedWake = lastWake + TASK_RATE_1KHZ;
        if (now > expectedWake)  // Only count as miss if actually late
        {
            TickType_t latency = now - expectedWake;
            deadline_miss_count[module]++;
            if (latency > worst_latency_ticks[module])
            {
                worst_latency_ticks[module] = latency;
            }
        }

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

#if FEATURE_AUTO_CONFIGURE_ADDRESS_FROM_UUID
    // Auto-configure UWB address based on device UUID if known
    platform_system_device_init();

    platform_system_device_info_t dev_info;
    if (platform_system_device_get_info(&dev_info))
    {
        if (dev_info.is_known_device)
        {
            uint16_t current_pan = uwb_get_pan_id();

            uwb_set_address(dev_info.assigned_address, current_pan);
        }
        else
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "app",
                              "Device not in mapping table (UUID word2: 0x%08lX)",
                              (unsigned long)dev_info.uuid_word2);
            error_handler_log(ERROR_SEVERITY_INFO, "app",
                              "Using default address 0x%04X. Add to config/device_mapping.c if needed",
                              uwb_get_address());
        }
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "app",
                          "Failed to initialize device ID system");
    }
#endif
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

uint32_t app_get_deadline_miss_count(modules_E module)
{
    if (module >= NUM_MODULES)
    {
        return 0;
    }
    return deadline_miss_count[module];
}

void app_get_deadline_stats(modules_E module, deadline_stats_t* stats)
{
    if (module >= NUM_MODULES || stats == NULL)
    {
        return;
    }

    stats->miss_count       = deadline_miss_count[module];
    stats->worst_latency_ms = worst_latency_ticks[module];
    stats->period_ms        = module_period_ms[module];
}
