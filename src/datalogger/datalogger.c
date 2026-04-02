/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "datalogger.h"
#include "FreeRTOS.h"
#include "common.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "task.h"
#include "task_config.h"
#include "protocol.pb.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define HEAP_LOW_MEMORY_THRESHOLD_BYTES 1024U
#define DATALOGGER_RATE_MS              1000U // 1 Hz

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_task(void* pvParameters);
STATIC void datalogger_refresh_task_cpu_usage(void);
STATIC void datalogger_monitor_heap_usage(void);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC float32_t cpu_usage[DATALOGGER_MAX_TASKS] = {0};
STATIC TaskStatus_t task_status_array[DATALOGGER_MAX_TASKS];
STATIC UBaseType_t num_tracked_tasks   = 0;
STATIC uint32_t current_free_heap      = 0U;
STATIC uint32_t minimum_ever_free_heap = 0U;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_refresh_task_cpu_usage(void)
{
    STATIC uint32_t prev_total_runtime                   = 0U;
    STATIC uint32_t prev_task_runtime[DATALOGGER_MAX_TASKS]     = {0};
    STATIC TaskHandle_t prev_task_handles[DATALOGGER_MAX_TASKS] = {NULL};

    uint32_t total_runtime = 0U;
    num_tracked_tasks      = uxTaskGetSystemState(task_status_array, DATALOGGER_MAX_TASKS, &total_runtime);
    if ((num_tracked_tasks == 0U) || (total_runtime == 0U))
    {
        return;
    }

    uint32_t total_runtime_change;
    if (total_runtime >= prev_total_runtime)
    {
        total_runtime_change = total_runtime - prev_total_runtime;
    }
    else
    {
        total_runtime_change = (UINT32_MAX - prev_total_runtime) + total_runtime + 1;
    }

    if (total_runtime_change == 0U)
    {
        return;
    }

    for (UBaseType_t task_index = 0U; task_index < num_tracked_tasks; ++task_index)
    {
        TaskHandle_t current_handle = task_status_array[task_index].xHandle;
        uint32_t current_runtime    = task_status_array[task_index].ulRunTimeCounter;

        uint32_t previous_runtime = 0U;
        for (uint8_t prev_index = 0U; prev_index < DATALOGGER_MAX_TASKS; ++prev_index)
        {
            if (prev_task_handles[prev_index] == current_handle)
            {
                previous_runtime = prev_task_runtime[prev_index];
                break;
            }
        }

        uint32_t runtime_change;
        if (current_runtime >= previous_runtime)
        {
            runtime_change = current_runtime - previous_runtime;
        }
        else
        {
            runtime_change = (UINT32_MAX - previous_runtime) + current_runtime + 1;
        }

        float32_t usage_percent = 0.0f;
        if (total_runtime_change != 0U)
        {
            usage_percent = (100.0f * (float32_t)runtime_change) / (float32_t)total_runtime_change;
        }

        cpu_usage[task_index] = usage_percent;

        prev_task_handles[task_index] = current_handle;
        prev_task_runtime[task_index] = current_runtime;
    }

    prev_total_runtime = total_runtime;
}


STATIC void datalogger_monitor_heap_usage(void)
{
    current_free_heap      = xPortGetFreeHeapSize();
    minimum_ever_free_heap = xPortGetMinimumEverFreeHeapSize();

    // Log warning if free heap is getting low (less than 1KB)
    if (current_free_heap < HEAP_LOW_MEMORY_THRESHOLD_BYTES)
    {
        DataloggerLowMemoryEvent ev = DataloggerLowMemoryEvent_init_zero;
        ev.bytes_free = current_free_heap;
        protocol_tx_DataloggerLowMemoryEvent(&ev);
    }
}

STATIC void datalogger_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        datalogger_monitor_heap_usage();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DATALOGGER_RATE_MS));
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementation
 *---------------------------------------------------------------------------*/

void datalogger_init(void)
{
    BaseType_t result = xTaskCreate(datalogger_task, "datalogger", TASK_STACK_2KB, NULL,
                                    TASK_PRIORITY_DATALOGGER, NULL);
    if (result != pdPASS)
    {
        system_halt("datalogger", "Failed to create datalogger task");
    }
}

uint32_t datalogger_get_task_usage(task_cpu_info_t* tasks, uint32_t max_tasks)
{
    if (tasks == NULL || max_tasks == 0)
    {
        return 0;
    }

    datalogger_refresh_task_cpu_usage();

    uint32_t count = (num_tracked_tasks < max_tasks) ? num_tracked_tasks : max_tasks;
    for (uint32_t i = 0; i < count; i++)
    {
        tasks[i].task_name   = task_status_array[i].pcTaskName;
        tasks[i].cpu_percent = cpu_usage[i];
    }

    return count;
}

void datalogger_get_system_stats(system_stats_t* stats, task_cpu_info_t* task_buffer,
                                 uint32_t max_tasks)
{
    if (stats == NULL)
    {
        return;
    }

    stats->current_free_heap      = current_free_heap;
    stats->minimum_ever_free_heap = minimum_ever_free_heap;
    stats->total_heap_size        = configTOTAL_HEAP_SIZE;
    stats->task_info              = task_buffer;

    if (task_buffer != NULL && max_tasks > 0)
    {
        stats->num_tasks = datalogger_get_task_usage(task_buffer, max_tasks);
    }
    else
    {
        stats->num_tasks = (uint32_t)uxTaskGetNumberOfTasks();
    }
}
