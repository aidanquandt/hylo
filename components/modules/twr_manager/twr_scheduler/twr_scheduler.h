#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TWR_SCHEDULER_MAX_TARGETS 8 // Maximum number of ranging targets to track

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    TWR_SCHED_STRATEGY_ROUND_ROBIN = 0, // Cycle through all targets sequentially
    TWR_SCHED_STRATEGY_PRIORITY,        // Reserved for future use
    TWR_SCHED_STRATEGY_ADAPTIVE         // Reserved for future use
} twr_scheduler_strategy_e;

typedef struct
{
    uint16_t address;
    uint8_t priority; // Reserved for priority-based scheduling
    bool enabled;     // Can be disabled without removing from list
    uint32_t success_count;
    uint32_t failure_count;
    uint8_t consecutive_failures; // Track consecutive failures for backoff
    uint32_t backoff_until_ms;    // Don't retry this target until this time
} twr_target_entry_t;

typedef struct
{
    uint8_t target_count;    // Number of targets in list
    uint8_t current_index;   // Current position in round-robin
    uint16_t current_target; // Currently selected target address
    twr_scheduler_strategy_e strategy;
} twr_scheduler_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the TWR scheduler
 */
void twr_scheduler_init(void);

/**
 * @brief Get the next target address to range with
 * @return Next target address, or 0x0000 if no targets configured
 */
uint16_t twr_scheduler_get_next_target(void);

/**
 * @brief Get the current target address (without advancing)
 * @return Current target address, or 0x0000 if no targets configured
 */
uint16_t twr_scheduler_get_current_target(void);

/**
 * @brief Add a target to the scheduling list
 * @param address Target address to add
 * @return true if added successfully, false if list is full or already exists
 */
bool twr_scheduler_add_target(uint16_t address);

/**
 * @brief Remove a target from the scheduling list
 * @param address Target address to remove
 * @return true if removed successfully, false if not found
 */
bool twr_scheduler_remove_target(uint16_t address);

/**
 * @brief Set all targets at once (replaces existing list)
 * @param addresses Array of target addresses
 * @param count Number of targets in array
 * @return true if set successfully, false if count exceeds maximum
 */
bool twr_scheduler_set_targets(const uint16_t* addresses, uint8_t count);

/**
 * @brief Clear all targets from the scheduling list
 */
void twr_scheduler_clear_all(void);

/**
 * @brief Enable or disable a specific target
 * @param address Target address
 * @param enabled true to enable, false to disable
 * @return true if successful, false if target not found
 */
bool twr_scheduler_set_target_enabled(uint16_t address, bool enabled);

/**
 * @brief Report ranging result to scheduler (for future adaptive strategies)
 * @param address Target address that was ranged
 * @param success true if ranging succeeded, false if failed
 */
void twr_scheduler_report_result(uint16_t address, bool success);

/**
 * @brief Set the scheduling strategy
 * @param strategy Strategy to use (currently only ROUND_ROBIN is implemented)
 */
void twr_scheduler_set_strategy(twr_scheduler_strategy_e strategy);

/**
 * @brief Get current scheduler status
 * @param status Pointer to status structure to fill
 */
void twr_scheduler_get_status(twr_scheduler_status_t* status);

/**
 * @brief Get the number of configured targets
 * @return Number of targets in the list
 */
uint8_t twr_scheduler_get_target_count(void);
