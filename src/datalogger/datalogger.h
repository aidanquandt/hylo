#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DATALOGGER_MAX_TASKS 64U

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    const char* task_name;
    float cpu_percent;
} task_cpu_info_t;

typedef struct
{
    uint32_t current_free_heap;
    uint32_t minimum_ever_free_heap;
    uint32_t total_heap_size;
    uint32_t num_tasks;
    task_cpu_info_t* task_info;
} system_stats_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void datalogger_init(void);
uint32_t datalogger_get_task_usage(task_cpu_info_t* tasks, uint32_t max_tasks);
void datalogger_get_system_stats(system_stats_t* stats, task_cpu_info_t* task_buffer,
                                 uint32_t max_tasks);
/** Idle-task CPU share (0–100 %) since last call; uses run-time stats clock (lightweight). */
float32_t datalogger_sample_idle_cpu_percent(void);