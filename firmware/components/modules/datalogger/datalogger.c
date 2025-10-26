/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "datalogger.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STATS_UPDATE_INTERVAL 50
#define MAX_TASKS 10
#define IDLE_CPU_FILTER_ALPHA 0.2f

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
    float32_t idle_cpu_filtered;
} task_cpu_usage_t;

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void);

/*---------------------------------------------------------------------------
 * Data declarations
 *---------------------------------------------------------------------------*/
STATIC TaskStatus_t task_status_array[MAX_TASKS];
STATIC task_cpu_usage_t cpu_usage = { 0.0f };

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_monitor_rtos_usage(void) {
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    if (num_tasks > MAX_TASKS) {
        num_tasks = MAX_TASKS;
    }
    
    uint32_t total_runtime;
    num_tasks = uxTaskGetSystemState(task_status_array, num_tasks, &total_runtime);
    
    if (total_runtime > 0U) {
        for (UBaseType_t i = 0; i < num_tasks; i++) {
            float32_t percentage = (float32_t)(task_status_array[i].ulRunTimeCounter) / (float32_t)total_runtime;
            
            if (strncmp(task_status_array[i].pcTaskName, "Datalogger", 10) == 0) {
                cpu_usage.datalogger_cpu = percentage;
            } else if (strncmp(task_status_array[i].pcTaskName, "Node", 4) == 0) {
                cpu_usage.node_cpu = percentage;
            } else if (strncmp(task_status_array[i].pcTaskName, "SensorFusion", 12) == 0) {
                cpu_usage.sensor_fusion_cpu = percentage;
            } else if (strncmp(task_status_array[i].pcTaskName, "TDMA", 4) == 0) {
                cpu_usage.tdma_cpu = percentage;
            } else if (strncmp(task_status_array[i].pcTaskName, "TWR", 3) == 0) {
                cpu_usage.twr_cpu = percentage;
            } else if (strncmp(task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
                cpu_usage.idle_cpu = percentage;
                cpu_usage.idle_cpu_filtered = (IDLE_CPU_FILTER_ALPHA * percentage) + ((1.0f - IDLE_CPU_FILTER_ALPHA) * cpu_usage.idle_cpu_filtered);
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