/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "datalogger.h"
#include "FreeRTOS.h"
#include "app.h"
#include "common.h"
#include "module.h"
#include "protocol_tx.h"
#include "task.h"
#include "protocol.pb.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DEADLINE_MISS_STARTUP_GRACE_PERIOD_S 5U
#define HEAP_LOW_MEMORY_THRESHOLD_BYTES 1024U

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);
STATIC void datalogger_monitor_heap_usage(void);
STATIC void datalogger_monitor_deadline_misses(void);
STATIC void datalogger_process_1Hz(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
extern const module_S datalogger_module;
const module_S datalogger_module = {
    .module_name        = "datalogger",
    .module_init        = NULL,
    .module_create_task = NULL,
    .module_process_1Hz = datalogger_process_1Hz,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};

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
STATIC void datalogger_monitor_rtos_usage(void)
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

STATIC void datalogger_monitor_deadline_misses(void)
{
    STATIC uint32_t prev_miss_count[NUM_MODULES] = {0};
    STATIC uint32_t startup_counter              = 0;

    // Skip reporting for first 5 seconds to allow modules to initialize
    if (startup_counter < DEADLINE_MISS_STARTUP_GRACE_PERIOD_S)
    {
        startup_counter++;
        // Initialize baseline after grace period
        if (startup_counter == DEADLINE_MISS_STARTUP_GRACE_PERIOD_S)
        {
            /* Establish baseline; worst_latency is also reset here (per-module). */
            for (modules_E module = (modules_E)0U; module < NUM_MODULES; module++)
            {
                prev_miss_count[module] = app_get_deadline_miss_count(module);
                app_reset_deadline_stats(module);
            }
        }
        return;
    }

    for (modules_E module = (modules_E)0U; module < NUM_MODULES; module++)
    {
        deadline_stats_t stats;
        app_get_deadline_stats(module, &stats);

        if (stats.miss_count > prev_miss_count[module])
        {
            uint32_t new_misses = stats.miss_count - prev_miss_count[module];
            DataloggerTimingMissesEvent ev = DataloggerTimingMissesEvent_init_zero;
            if (modules[module]->module_name != NULL)
            {
                strncpy(ev.task_name, modules[module]->module_name, sizeof(ev.task_name) - 1);
                ev.task_name[sizeof(ev.task_name) - 1] = '\0';
            }
            ev.miss_count      = new_misses;
            ev.deadline_ms     = stats.period_ms;
            ev.worst_latency_ms = stats.worst_latency_ms;
            protocol_tx_DataloggerTimingMissesEvent(&ev);
            prev_miss_count[module] = stats.miss_count;
        }
    }
}

STATIC void datalogger_process_1Hz(void)
{
    datalogger_monitor_rtos_usage();
    datalogger_monitor_heap_usage();
    datalogger_monitor_deadline_misses();
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

/*---------------------------------------------------------------------------
 * Public Function Implementation
 *---------------------------------------------------------------------------*/

uint32_t datalogger_get_task_usage(task_cpu_info_t* tasks, uint32_t max_tasks)
{
    if (tasks == NULL || max_tasks == 0)
    {
        return 0;
    }

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
        stats->num_tasks = 0;
    }
}
