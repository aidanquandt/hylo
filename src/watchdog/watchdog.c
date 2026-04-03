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

#define WATCHDOG_EXPECTED_GOOD_TICKS_PER_CHECK \
    ((uint32_t)((WATCHDOG_CHECK_RATE_MS * (uint32_t)configTICK_RATE_HZ) / 1000U))

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// 1 kHz task: counts wakes where xTaskGetTickCount advanced by exactly one since last wake.
STATIC volatile uint32_t watchdog_1khz_good_count = 0;
// Times xTaskGetTickCount advanced by other than one while scheduler was not suspended — latched per check window.
STATIC volatile uint32_t watchdog_tick_gap_count = 0;

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
    uint32_t   last_good_snapshot = 0;
    uint8_t    have_good_baseline = 0;

#if FEATURE_WATCHDOG_ENABLE_IWDG
    watchdog_driver_refresh();
#endif

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WATCHDOG_CHECK_RATE_MS));

        uint32_t gaps;
        uint32_t good_now;
        taskENTER_CRITICAL();
        gaps                    = watchdog_tick_gap_count;
        watchdog_tick_gap_count = 0;
        good_now                = watchdog_1khz_good_count;
        taskEXIT_CRITICAL();

        if (!have_good_baseline)
        {
            last_good_snapshot   = good_now;
            have_good_baseline   = 1;
#if FEATURE_WATCHDOG_ENABLE_IWDG
            watchdog_driver_refresh();
#endif
            continue;
        }

        uint32_t       good_delta = good_now - last_good_snapshot;
        const uint32_t expected   = WATCHDOG_EXPECTED_GOOD_TICKS_PER_CHECK;
        last_good_snapshot        = good_now;

        const bool good_ticks_in_range =
            (good_delta + 1U >= expected) && (good_delta <= expected + 1U);
        const bool ok = (gaps == 0U) && good_ticks_in_range;

        if (ok)
        {
#if FEATURE_WATCHDOG_ENABLE_IWDG
            watchdog_driver_refresh();
#endif
        }
        else
        {
            WatchdogTaskFailureEvent ev = WatchdogTaskFailureEvent_init_zero;
            ev.current_heartbeats  = good_delta;
            ev.expected_heartbeats = expected;
            ev.tick_gap_count      = gaps;
            protocol_tx_WatchdogTaskFailureEvent(&ev);
        }
    }
}

STATIC void watchdog_1khz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t prevTick = 0;
    uint8_t    havePrev = 0;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));

        TickType_t now = xTaskGetTickCount();

        if (havePrev)
        {
            TickType_t delta = (TickType_t)(now - prevTick);
            if (delta == 1U)
            {
                taskENTER_CRITICAL();
                watchdog_1khz_good_count++;
                taskEXIT_CRITICAL();
            }
            else
            {
                taskENTER_CRITICAL();
                watchdog_tick_gap_count++;
                taskEXIT_CRITICAL();
            }
        }

        prevTick = now;
        havePrev = 1;
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
                         TASK_PRIORITY_WATCHDOG, NULL);
    if (result != pdPASS)
    {
        system_halt("watchdog", "Failed to create 1kHz task");
    }
}
