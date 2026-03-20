/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "common.h"
#include <stdio.h>
#include "datalogger.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "protocol.pb.h"
#include "module.h"
#include "gpio_driver.h"
#include "os_driver.h"
#include "system_driver.h"
#include "sensor_fusion.h"
#include "task.h"
#include "uart_driver.h"
#include "uart_protocol_task.h"
#include "uwb.h"
#include "wifi.h"
#include "sdcard_driver.h"

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
            // Snap forward after a large overrun to avoid a catch-up storm of rapid-fire ticks
            if (latency > TASK_RATE_1HZ)
            {
                lastWake = now;
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
            if (latency > TASK_RATE_10HZ)
            {
                lastWake = now;
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
            if (latency > TASK_RATE_100HZ)
            {
                lastWake = now;
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
            // Snap forward after a large overrun (e.g. IMU init) to avoid a flood of catch-up ticks
            if (latency > TASK_RATE_1KHZ * 5)
            {
                lastWake = now;
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
                system_halt("app", "Failed to create task (1kHz)");
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
                system_halt("app", "Failed to create task (100Hz)");
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
                system_halt("app", "Failed to create task (10Hz)");
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
                system_halt("app", "Failed to create task (1Hz)");
            }
        }
    }
}

STATIC void app_post_module_initialization(void)
{
#if FEATURE_AUTO_CONFIGURE_ADDRESS_FROM_UUID
    // Auto-configure UWB address based on device UUID if known
    system_driver_device_init();

    system_driver_device_info_t dev_info;
    if (system_driver_device_get_info(&dev_info))
    {
        if (dev_info.is_known_device)
        {
            uint16_t current_pan = uwb_get_pan_id();

            uwb_set_address(dev_info.assigned_address, current_pan);
        }
        else
        {
            AppDeviceNotInMappingTableEvent ev = AppDeviceNotInMappingTableEvent_init_zero;
            ev.uuid_word2 = dev_info.uuid_word2;
            protocol_tx_AppDeviceNotInMappingTableEvent(&ev);
            AppUsingDefaultAddressEvent ev2 = AppUsingDefaultAddressEvent_init_zero;
            ev2.address = uwb_get_address();
            protocol_tx_AppUsingDefaultAddressEvent(&ev2);
        }
    }
    else
    {
        AppFailedToInitDeviceIdEvent ev = AppFailedToInitDeviceIdEvent_init_zero;
        protocol_tx_AppFailedToInitDeviceIdEvent(&ev);
    }
#endif
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/

void app_init(void)
{
    uart_driver_init();
    uart_protocol_task_start();
    app_initialize_modules();
    wifi_start_task();
    app_create_module_tasks();
    app_post_module_initialization();
    sdcard_driver_init();
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

void app_reset_deadline_stats(modules_E module)
{
    if (module >= NUM_MODULES)
    {
        return;
    }

    worst_latency_ticks[module] = 0;
}
