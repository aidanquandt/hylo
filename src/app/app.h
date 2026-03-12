#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "module.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint32_t miss_count;       // Total number of deadline misses
    uint32_t worst_latency_ms; // Worst-case latency in milliseconds
    uint32_t period_ms;        // Expected period (deadline) in milliseconds
} deadline_stats_t;

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/
void app_init(void);
uint32_t app_get_deadline_miss_count(modules_E module);
void app_get_deadline_stats(modules_E module, deadline_stats_t* stats);
void app_reset_deadline_stats(modules_E module);