/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "datalogger.h"
#include "common.h"
#include "module.h"
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
 * Module Functions
 *---------------------------------------------------------------------------*/
extern const module_S datalogger_module;
const module_S datalogger_module = {
        .module_init = datalogger_init,
        .module_process_100Hz = datalogger_process,
};

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/

STATIC void datalogger_monitor_rtos_usage(void);

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/
STATIC TaskStatus_t task_status_array[NUM_MODULES + 5U]; // Aidan - why are there 6 tasks - should have 4 (4x freertos) + 1 idle? what is other? maybe scheduler?
STATIC task_cpu_usage_t cpu_usage = { 0.0f };
STATIC task_name_map_t map[] = {
    { TOSTRING(DATALOGGER_MODULE), STRLEN_LITERAL(TOSTRING(DATALOGGER_MODULE)), &cpu_usage.datalogger_cpu },
    { TOSTRING(NODE_MODULE),       STRLEN_LITERAL(TOSTRING(NODE_MODULE)),       &cpu_usage.node_cpu },
    { TOSTRING(TDMA_MODULE),       STRLEN_LITERAL(TOSTRING(TDMA_MODULE)),       &cpu_usage.tdma_cpu },
    { TOSTRING(TWR_MODULE),        STRLEN_LITERAL(TOSTRING(TWR_MODULE)),        &cpu_usage.twr_cpu },
    { "IDLE",                      STRLEN_LITERAL("IDLE"),                      &cpu_usage.idle_cpu }
};

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void) {
    const size_t map_size = sizeof(map) / sizeof(map[0]);
    uint32_t total_runtime;

    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    if (num_tasks > (NUM_MODULES + 5U)) {
        num_tasks = (NUM_MODULES + 5U);
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