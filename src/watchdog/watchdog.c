/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "watchdog.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "protocol.pb.h"
#include "task_config.h"
#include "watchdog_driver.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define WATCHDOG_CHECK_RATE_MS 1000U // Check heartbeats every 1 second
#define PRIORITY_WATCHDOG_TASK 0     // Lowest priority - only runs if all others can run

// Heartbeat bitmask - one bit per task rate
#define HEARTBEAT_1KHZ_BIT (1U << 0)
#define HEARTBEAT_100HZ_BIT (1U << 1)
#define HEARTBEAT_10HZ_BIT (1U << 2)
#define HEARTBEAT_1HZ_BIT (1U << 3)
#define HEARTBEAT_ALL_BITS                                                                         \
    (HEARTBEAT_1KHZ_BIT | HEARTBEAT_100HZ_BIT | HEARTBEAT_10HZ_BIT | HEARTBEAT_1HZ_BIT)

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC volatile uint32_t task_heartbeats = 0; // Bitmask of which tasks have run since last check

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument);

/*---------------------------------------------------------------------------
 * Private Function Prototypes (module-internal)
 *---------------------------------------------------------------------------*/
STATIC void watchdog_process_1khz(void);
STATIC void watchdog_process_100hz(void);
STATIC void watchdog_process_10hz(void);
STATIC void watchdog_process_1hz(void);
STATIC void watchdog_1khz_task(void* pvParameters);
STATIC void watchdog_100hz_task(void* pvParameters);
STATIC void watchdog_10hz_task(void* pvParameters);
STATIC void watchdog_1hz_task(void* pvParameters);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();

#if FEATURE_WATCHDOG_ENABLE_IWDG
    watchdog_driver_refresh();
#endif

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WATCHDOG_CHECK_RATE_MS));

        uint32_t current_heartbeats = task_heartbeats;
        task_heartbeats             = 0; // Clear for next check

        // Only refresh if all tasks are alive
        if (current_heartbeats == HEARTBEAT_ALL_BITS)
        {
#if FEATURE_WATCHDOG_ENABLE_IWDG
            watchdog_driver_refresh();
#endif
        }
        else
        {
            // Task failure - send event, DON'T refresh; system will reset in ~8s via IWDG
            WatchdogTaskFailureEvent ev = WatchdogTaskFailureEvent_init_zero;
            ev.current_heartbeats  = current_heartbeats;
            ev.expected_heartbeats = HEARTBEAT_ALL_BITS;
            protocol_tx_WatchdogTaskFailureEvent(&ev);
            // Stay in loop without refreshing until IWDG resets system
        }
    }
}

STATIC void watchdog_1khz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        watchdog_process_1khz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

STATIC void watchdog_100hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        watchdog_process_100hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

STATIC void watchdog_10hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        watchdog_process_10hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

STATIC void watchdog_1hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        watchdog_process_1hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
}

void watchdog_init(void)
{
    BaseType_t result;

    result = xTaskCreate(watchdog_task, "watchdog", TASK_STACK_1KB, NULL,
                         PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create watchdog task");
    }

    result = xTaskCreate(watchdog_1khz_task, "wdog_1khz", TASK_STACK_1KB, NULL,
                         PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 1kHz task");
    }

    result = xTaskCreate(watchdog_100hz_task, "wdog_100hz", TASK_STACK_1KB, NULL,
                         PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 100Hz task");
    }

    result = xTaskCreate(watchdog_10hz_task, "wdog_10hz", TASK_STACK_1KB, NULL,
                         PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 10Hz task");
    }

    result = xTaskCreate(watchdog_1hz_task, "wdog_1hz", TASK_STACK_1KB, NULL,
                         PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 1Hz task");
    }
}

STATIC void watchdog_process_1khz(void)
{
    task_heartbeats |= HEARTBEAT_1KHZ_BIT;
}

STATIC void watchdog_process_100hz(void)
{
    task_heartbeats |= HEARTBEAT_100HZ_BIT;
}

STATIC void watchdog_process_10hz(void)
{
    task_heartbeats |= HEARTBEAT_10HZ_BIT;
}

STATIC void watchdog_process_1hz(void)
{
    task_heartbeats |= HEARTBEAT_1HZ_BIT;
}
