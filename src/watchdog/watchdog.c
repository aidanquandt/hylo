/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "watchdog.h"
#include "FreeRTOS.h"
#include "error_handler.h"
#include "feature_config.h"
#include "module.h"
#include "platform_watchdog.h"
#include "semphr.h"
#include "task.h"
#include "task_config.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define WATCHDOG_CHECK_RATE_MS 1000U // Check heartbeats every 1 second
#define MAX_MONITORED_TASKS 16       // Maximum tasks we can monitor

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    TaskHandle_t task_handle;
    const char* task_name;
    uint32_t expected_period_ms;
    TickType_t last_heartbeat_tick;
    bool registered;
} monitored_task_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC monitored_task_t monitored_tasks[MAX_MONITORED_TASKS] = {0};
STATIC uint32_t num_monitored_tasks                          = 0;
STATIC SemaphoreHandle_t watchdog_mutex                      = NULL;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void watchdog_init(void);
STATIC void watchdog_create_tasks(void);

extern const module_S watchdog_module;

const module_S watchdog_module = {
    .module_name         = "watchdog",
    .module_init         = watchdog_init,
    .module_create_tasks = watchdog_create_tasks,
};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void watchdog_task(void* argument)
{
    (void)argument;

    TickType_t lastWake = xTaskGetTickCount();

#if FEATURE_WATCHDOG_ENABLE_IWDG
    platform_watchdog_refresh();
#endif

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(WATCHDOG_CHECK_RATE_MS));

        bool all_tasks_healthy  = true;
        TickType_t current_tick = xTaskGetTickCount();

        // Check all monitored tasks
        xSemaphoreTake(watchdog_mutex, portMAX_DELAY);

        for (uint32_t i = 0; i < num_monitored_tasks; i++)
        {
            if (!monitored_tasks[i].registered)
            {
                continue;
            }

            TickType_t elapsed     = current_tick - monitored_tasks[i].last_heartbeat_tick;
            TickType_t max_allowed = pdMS_TO_TICKS(monitored_tasks[i].expected_period_ms) +
                                     pdMS_TO_TICKS(WATCHDOG_CHECK_RATE_MS);

            if (elapsed > max_allowed)
            {
                error_handler_log(
                    ERROR_SEVERITY_FATAL, "watchdog",
                    "Task '%s' missed heartbeat! Elapsed: %lu ms (max: %lu ms) - system will reset",
                    monitored_tasks[i].task_name, (unsigned long)(elapsed * portTICK_PERIOD_MS),
                    (unsigned long)(monitored_tasks[i].expected_period_ms +
                                    WATCHDOG_CHECK_RATE_MS));
                all_tasks_healthy = false;
            }
        }

        xSemaphoreGive(watchdog_mutex);

        // Only refresh hardware watchdog if all tasks are healthy
        if (all_tasks_healthy)
        {
#if FEATURE_WATCHDOG_ENABLE_IWDG
            platform_watchdog_refresh();
#endif
        }
        else
        {
            // Don't refresh - system will reset via hardware watchdog in ~8s
            // Stay in loop to continue logging (if possible)
        }
    }
}

STATIC void watchdog_init(void)
{
    // Create mutex for thread-safe access to monitored tasks
    watchdog_mutex = xSemaphoreCreateMutex();
    if (watchdog_mutex == NULL)
    {
        error_handler_fatal("watchdog", "Failed to create mutex");
    }
}

STATIC void watchdog_create_tasks(void)
{
    BaseType_t result = xTaskCreate(watchdog_task, "watchdog", TASK_STACK_SMALL, NULL,
                                    TASK_PRIORITY_WATCHDOG, NULL);
    if (result != pdPASS)
    {
        error_handler_fatal("watchdog", "Failed to create watchdog task");
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void watchdog_register_task(uint32_t expected_period_ms)
{
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    const char* task_name     = pcTaskGetName(current_task);

    xSemaphoreTake(watchdog_mutex, portMAX_DELAY);

    if (num_monitored_tasks >= MAX_MONITORED_TASKS)
    {
        xSemaphoreGive(watchdog_mutex);
        error_handler_fatal("watchdog", "Too many monitored tasks (max: %d)", MAX_MONITORED_TASKS);
        return;
    }

    // Check if already registered
    for (uint32_t i = 0; i < num_monitored_tasks; i++)
    {
        if (monitored_tasks[i].task_handle == current_task)
        {
            xSemaphoreGive(watchdog_mutex);
            return; // Already registered
        }
    }

    // Register new task
    monitored_tasks[num_monitored_tasks].task_handle         = current_task;
    monitored_tasks[num_monitored_tasks].task_name           = task_name;
    monitored_tasks[num_monitored_tasks].expected_period_ms  = expected_period_ms;
    monitored_tasks[num_monitored_tasks].last_heartbeat_tick = xTaskGetTickCount();
    monitored_tasks[num_monitored_tasks].registered          = true;
    num_monitored_tasks++;

    xSemaphoreGive(watchdog_mutex);
}

void watchdog_heartbeat(void)
{
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

    xSemaphoreTake(watchdog_mutex, portMAX_DELAY);

    // Find and update heartbeat for current task
    for (uint32_t i = 0; i < num_monitored_tasks; i++)
    {
        if (monitored_tasks[i].task_handle == current_task && monitored_tasks[i].registered)
        {
            monitored_tasks[i].last_heartbeat_tick = xTaskGetTickCount();
            break;
        }
    }

    xSemaphoreGive(watchdog_mutex);
}
