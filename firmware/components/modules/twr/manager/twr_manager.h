#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "../scheduler/twr_scheduler.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Start the TWR manager and begin ranging operations
 * @return true if started successfully, false otherwise
 * @note Ensure anchors are configured via twr_scheduler API before starting
 */
bool twr_manager_start(void);

/**
 * @brief Stop the TWR manager and cancel any ongoing ranging
 */
void twr_manager_stop(void);

/**
 * @brief Check if the TWR manager is actively ranging
 * @return true if ranging or backing off, false if idle or faulted
 */
bool twr_manager_is_active(void);

/**
 * @brief Get the total number of successful ranges
 * @return Success count
 */
uint32_t twr_manager_get_success_count(void);

/**
 * @brief Get the total number of failed ranges
 * @return Failure count
 */
uint32_t twr_manager_get_failure_count(void);

/*---------------------------------------------------------------------------
 * Scheduler Pass-through Functions (for convenience)
 * Note: You can also use twr_scheduler API directly
 *---------------------------------------------------------------------------*/

/**
 * @brief Add an anchor to the ranging schedule
 * @param address Anchor address to add
 * @return true if added successfully
 */
static inline bool twr_manager_add_anchor(uint16_t address)
{
    return twr_scheduler_add_anchor(address);
}

/**
 * @brief Remove an anchor from the ranging schedule
 * @param address Anchor address to remove
 * @return true if removed successfully
 */
static inline bool twr_manager_remove_anchor(uint16_t address)
{
    return twr_scheduler_remove_anchor(address);
}

/**
 * @brief Set all anchors at once (replaces existing list)
 * @param addresses Array of anchor addresses
 * @param count Number of anchors
 * @return true if set successfully
 */
static inline bool twr_manager_set_anchors(const uint16_t* addresses, uint8_t count)
{
    return twr_scheduler_set_anchors(addresses, count);
}

/**
 * @brief Get the number of configured anchors
 * @return Anchor count
 */
static inline uint8_t twr_manager_get_anchor_count(void)
{
    return twr_scheduler_get_anchor_count();
}
