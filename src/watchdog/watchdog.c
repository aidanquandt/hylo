/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "watchdog.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "uart_protocol.pb.h"
#include "module.h"
#include "watchdog_driver.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define WATCHDOG_CHECK_RATE_MS 1000U // Check heartbeats every 1 second
#define PRIORITY_WATCHDOG_TASK 0     // Lowest priority - only runs if all others can run
#define TASK_STACK_SMALL 256

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
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void watchdog_init(void);
STATIC void watchdog_create_task(void);
STATIC void watchdog_process_1khz(void);
STATIC void watchdog_process_100hz(void);
STATIC void watchdog_process_10hz(void);
STATIC void watchdog_process_1hz(void);

extern const module_S watchdog_module;

const module_S watchdog_module = {
    .module_name          = "watchdog",
    .module_init          = watchdog_init,
    .module_create_task   = watchdog_create_task,
    .module_process_1Hz   = watchdog_process_1hz,
    .module_process_10Hz  = watchdog_process_10hz,
    .module_process_100Hz = watchdog_process_100hz,
    .module_process_1kHz  = watchdog_process_1khz,
};

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

STATIC void watchdog_init(void)
{
    // Module initialization (called before RTOS starts)
    // Nothing to do here - watchdog task will init IWDG
}

STATIC void watchdog_create_task(void)
{
    BaseType_t result = xTaskCreate(watchdog_task, "watchdog", TASK_STACK_SMALL, NULL,
                                    PRIORITY_WATCHDOG_TASK, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create watchdog task");
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
