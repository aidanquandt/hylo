/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "watchdog.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "protocol.pb.h"
#include <stdbool.h>
#include "task_config.h"
#include "watchdog_driver.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define WATCHDOG_CHECK_RATE_MS 1000U // How often supervisor refreshes IWDG / reports

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
/** Set if tick count jumped by more than 1 between canary runs (missed scheduling / long mask). */
STATIC volatile uint32_t heartbeat_tick_gap_fault = 0U;
/** Observed tick delta on failure (for WatchdogTaskFailureEvent). */
STATIC volatile uint32_t heartbeat_last_bad_elapsed = 0U;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument);
STATIC void watchdog_1khz_task(void* pvParameters);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();
    bool       failure_reported = false;

#if FEATURE_WATCHDOG_ENABLE_IWDG
    watchdog_driver_refresh();
#endif

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WATCHDOG_CHECK_RATE_MS));

        if (heartbeat_tick_gap_fault == 0U)
        {
#if FEATURE_WATCHDOG_ENABLE_IWDG
            watchdog_driver_refresh();
#endif
        }
        else if (!failure_reported)
        {
            /* One shot: repeated TX competes with the canary at low priority and hides the real delta. */
            WatchdogTaskFailureEvent ev = WatchdogTaskFailureEvent_init_zero;
            ev.current_heartbeats  = heartbeat_last_bad_elapsed;
            ev.expected_heartbeats = 1U;
            protocol_tx_WatchdogTaskFailureEvent(&ev);
            failure_reported = true;
        }
    }
}

STATIC void watchdog_1khz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    bool       have_prev = false;
    TickType_t prevTick  = 0;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
        TickType_t now = xTaskGetTickCount();

        if (have_prev)
        {
            TickType_t elapsed = (TickType_t)(now - prevTick);
            /* Only "too many ticks" is a fault. elapsed == 0 is normal: vTaskDelayUntil does
             * not sleep when the wake time is already past; it advances lastWake and returns,
             * so several loop bodies can run with the same xTaskGetTickCount() (catch-up). */
            if (elapsed > (TickType_t)1)
            {
                heartbeat_last_bad_elapsed = (uint32_t)elapsed;
                heartbeat_tick_gap_fault   = 1U;
            }
        }

        have_prev = true;
        prevTick  = now;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void watchdog_init(void)
{
    BaseType_t result;

    result = xTaskCreate(watchdog_task, "watchdog", TASK_STACK_1KB, NULL,
                         TASK_PRIORITY_WATCHDOG, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create watchdog task");
    }

    result = xTaskCreate(watchdog_1khz_task, "wdog_1khz", TASK_STACK_1KB, NULL,
                         TASK_PRIORITY_WATCHDOG_CANARY, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 1kHz task");
    }
}
