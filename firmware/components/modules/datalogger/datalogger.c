/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "datalogger.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct {
    float32_t datalogger_cpu;
    float32_t node_cpu;
    float32_t sensor_fusion_cpu;
    float32_t tdma_cpu;
    float32_t twr_cpu;
    float32_t idle_cpu;
} task_cpu_usage_t;

typedef struct {
    const char *name;
    size_t name_len;
    float32_t *usage_ptr;
} task_name_map_t;

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);

/*---------------------------------------------------------------------------
 * Data declarations
 *---------------------------------------------------------------------------*/
STATIC TaskStatus_t task_status_array[NUM_MODULES];
STATIC task_cpu_usage_t cpu_usage = { 0.0f };
STATIC task_name_map_t map[] = {
    { "Datalogger",   10, &cpu_usage.datalogger_cpu },
    { "Node",          4, &cpu_usage.node_cpu },
    { "TDMA",          4, &cpu_usage.tdma_cpu },
    { "TWR",           3, &cpu_usage.twr_cpu },
    { "IDLE",          4, &cpu_usage.idle_cpu }
};

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void) {
    const size_t map_size = sizeof(map) / sizeof(map[0]);
    uint32_t total_runtime;

    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    if (num_tasks > NUM_MODULES) {
        num_tasks = NUM_MODULES;
    }

    num_tasks = uxTaskGetSystemState(task_status_array, num_tasks, &total_runtime);

    if (total_runtime > 0U) 
    {
        for (UBaseType_t task_idx = 0; task_idx < num_tasks; task_idx++) 
        {
            float32_t percentage = (float32_t)(task_status_array[task_idx].ulRunTimeCounter) / (float32_t)total_runtime;
            for (size_t map_idx = 0; map_idx < map_size; map_idx++) 
            {
                if (strncmp(task_status_array[task_idx].pcTaskName, map[map_idx].name, map[map_idx].name_len) == 0) 
                {
                    *(map[map_idx].usage_ptr) = percentage;
                }
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void datalogger_init(void) {
    // init
}

void datalogger_process(void) {
    datalogger_monitor_rtos_usage();
}