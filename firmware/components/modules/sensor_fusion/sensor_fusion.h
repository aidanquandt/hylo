#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "sensor_fusion_types.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
sensor_fusion_status_e sensor_fusion_push_event(const sensor_event_t* event);
sensor_fusion_status_e sensor_fusion_pop_event(sensor_event_t* event);
void sensor_fusion_get_stats(sensor_queue_stats_t* stats);
bool sensor_fusion_get_position(sensor_fusion_position_t* position);
void sensor_fusion_reset(void);