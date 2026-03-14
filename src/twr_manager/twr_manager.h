#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "twr_scheduler.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void twr_manager_init(void);
bool twr_manager_start(void);
void twr_manager_stop(void);
bool twr_manager_is_active(void);
uint32_t twr_manager_get_success_count(void);
uint32_t twr_manager_get_failure_count(void);
bool twr_manager_set_ranging_rate(uint16_t rate_hz);
uint16_t twr_manager_get_ranging_rate(void);

static inline bool twr_manager_add_target(uint16_t address)
{
    return twr_scheduler_add_target(address);
}
static inline bool twr_manager_remove_target(uint16_t address)
{
    return twr_scheduler_remove_target(address);
}
static inline bool twr_manager_set_targets(const uint16_t* addresses, uint8_t count)
{
    return twr_scheduler_set_targets(addresses, count);
}
static inline uint8_t twr_manager_get_target_count(void)
{
    return twr_scheduler_get_target_count();
}

