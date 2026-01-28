#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TWR_SCHEDULER_MAX_ANCHORS 8 // Maximum number of anchors to track

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    TWR_SCHED_STRATEGY_ROUND_ROBIN = 0, // Cycle through all anchors sequentially
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
} twr_anchor_entry_t;

typedef struct
{
    uint8_t anchor_count;    // Number of anchors in list
    uint8_t current_index;   // Current position in round-robin
    uint16_t current_target; // Currently selected anchor address
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
 * @brief Get the next anchor address to range with
 * @return Next anchor address, or 0x0000 if no anchors configured
 */
uint16_t twr_scheduler_get_next_target(void);

/**
 * @brief Get the current anchor address (without advancing)
 * @return Current anchor address, or 0x0000 if no anchors configured
 */
uint16_t twr_scheduler_get_current_target(void);

/**
 * @brief Add an anchor to the scheduling list
 * @param address Anchor address to add
 * @return true if added successfully, false if list is full or already exists
 */
bool twr_scheduler_add_anchor(uint16_t address);

/**
 * @brief Remove an anchor from the scheduling list
 * @param address Anchor address to remove
 * @return true if removed successfully, false if not found
 */
bool twr_scheduler_remove_anchor(uint16_t address);

/**
 * @brief Set all anchors at once (replaces existing list)
 * @param addresses Array of anchor addresses
 * @param count Number of anchors in array
 * @return true if set successfully, false if count exceeds maximum
 */
bool twr_scheduler_set_anchors(const uint16_t* addresses, uint8_t count);

/**
 * @brief Clear all anchors from the scheduling list
 */
void twr_scheduler_clear_all(void);

/**
 * @brief Enable or disable a specific anchor
 * @param address Anchor address
 * @param enabled true to enable, false to disable
 * @return true if successful, false if anchor not found
 */
bool twr_scheduler_set_anchor_enabled(uint16_t address, bool enabled);

/**
 * @brief Report ranging result to scheduler (for future adaptive strategies)
 * @param address Anchor address that was ranged
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
 * @brief Get the number of configured anchors
 * @return Number of anchors in the list
 */
uint8_t twr_scheduler_get_anchor_count(void);
