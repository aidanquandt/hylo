/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_scheduler.h"
#include "error_handler.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC twr_anchor_entry_t anchor_list[TWR_SCHEDULER_MAX_ANCHORS];
STATIC uint8_t anchor_count;
STATIC uint8_t current_index;
STATIC twr_scheduler_strategy_e strategy;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t find_anchor_index(uint16_t address);
STATIC uint16_t get_next_round_robin(void);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC int8_t find_anchor_index(uint16_t address)
{
    for (uint8_t i = 0; i < anchor_count; i++)
    {
        if (anchor_list[i].address == address)
        {
            return (int8_t)i;
        }
    }
    return -1; // Not found
}

STATIC uint16_t get_next_round_robin(void)
{
    if (anchor_count == 0)
    {
        return 0x0000;
    }

    // Find next enabled anchor
    uint8_t attempts = 0;
    while (attempts < anchor_count)
    {
        // Advance to next index
        current_index = (current_index + 1) % anchor_count;

        // Check if this anchor is enabled
        if (anchor_list[current_index].enabled)
        {
            return anchor_list[current_index].address;
        }

        attempts++;
    }

    // All anchors disabled - return 0x0000
    return 0x0000;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void twr_scheduler_init(void)
{
    memset(anchor_list, 0, sizeof(anchor_list));
    anchor_count  = 0;
    current_index = 0;
    strategy      = TWR_SCHED_STRATEGY_ROUND_ROBIN;
}

uint16_t twr_scheduler_get_next_target(void)
{
    if (anchor_count == 0)
    {
        return 0x0000; // No anchors configured
    }

    uint16_t next_address = 0x0000;

    switch (strategy)
    {
        case TWR_SCHED_STRATEGY_ROUND_ROBIN:
            next_address = get_next_round_robin();
            break;

        case TWR_SCHED_STRATEGY_PRIORITY:
        case TWR_SCHED_STRATEGY_ADAPTIVE:
            // Future implementation
            error_handler_log(ERROR_SEVERITY_WARNING, "twr_sched", "Strategy not implemented yet");
            next_address = get_next_round_robin(); // Fallback
            break;

        default:
            next_address = get_next_round_robin();
            break;
    }

    return next_address;
}

uint16_t twr_scheduler_get_current_target(void)
{
    if (anchor_count == 0)
    {
        return 0x0000;
    }

    // Bounds check
    if (current_index >= anchor_count)
    {
        current_index = 0; // Reset to start
    }

    // Return current anchor without advancing
    if (anchor_list[current_index].enabled)
    {
        return anchor_list[current_index].address;
    }

    // If current is disabled, get next valid one
    return twr_scheduler_get_next_target();
}

bool twr_scheduler_add_anchor(uint16_t address)
{
    // Check if list is full
    if (anchor_count >= TWR_SCHEDULER_MAX_ANCHORS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "Anchor list full");
        return false;
    }

    // Check if already exists
    if (find_anchor_index(address) >= 0)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "twr_sched", "Anchor 0x%04X already exists",
                          address);
        return false;
    }

    // Add new anchor
    anchor_list[anchor_count].address       = address;
    anchor_list[anchor_count].priority      = 0;
    anchor_list[anchor_count].enabled       = true;
    anchor_list[anchor_count].success_count = 0;
    anchor_list[anchor_count].failure_count = 0;
    anchor_count++;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Added anchor 0x%04X (total: %d)", address,
                      anchor_count);

    return true;
}

bool twr_scheduler_remove_anchor(uint16_t address)
{
    int8_t index = find_anchor_index(address);
    if (index < 0)
    {
        return false; // Not found
    }

    // Shift remaining anchors down
    for (uint8_t i = index; i < anchor_count - 1; i++)
    {
        anchor_list[i] = anchor_list[i + 1];
    }

    anchor_count--;

    // Clear last entry
    memset(&anchor_list[anchor_count], 0, sizeof(twr_anchor_entry_t));

    // Adjust current index if needed
    if (current_index >= anchor_count && anchor_count > 0)
    {
        current_index = 0; // Wrap around
    }

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Removed anchor 0x%04X (remaining: %d)",
                      address, anchor_count);

    return true;
}

bool twr_scheduler_set_anchors(const uint16_t* addresses, uint8_t count)
{
    if (addresses == NULL)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "NULL addresses pointer");
        return false;
    }

    if (count > TWR_SCHEDULER_MAX_ANCHORS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "Anchor count %d exceeds maximum %d",
                          count, TWR_SCHEDULER_MAX_ANCHORS);
        return false;
    }

    // Clear existing list
    twr_scheduler_clear_all();

    // Add all anchors
    for (uint8_t i = 0; i < count; i++)
    {
        anchor_list[i].address       = addresses[i];
        anchor_list[i].priority      = 0;
        anchor_list[i].enabled       = true;
        anchor_list[i].success_count = 0;
        anchor_list[i].failure_count = 0;
    }

    anchor_count  = count;
    current_index = 0;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Set %d anchors", count);

    return true;
}

void twr_scheduler_clear_all(void)
{
    // Only clear entries that were actually used
    if (anchor_count > 0)
    {
        memset(anchor_list, 0, anchor_count * sizeof(twr_anchor_entry_t));
    }
    anchor_count  = 0;
    current_index = 0;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Cleared all anchors");
}

bool twr_scheduler_set_anchor_enabled(uint16_t address, bool enabled)
{
    int8_t index = find_anchor_index(address);
    if (index < 0)
    {
        return false;
    }

    anchor_list[index].enabled = enabled;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Anchor 0x%04X %s", address,
                      enabled ? "enabled" : "disabled");

    return true;
}

void twr_scheduler_report_result(uint16_t address, bool success)
{
    int8_t index = find_anchor_index(address);
    if (index < 0)
    {
        return; // Anchor not found - silently ignore
    }

    if (success)
    {
        anchor_list[index].success_count++;
    }
    else
    {
        anchor_list[index].failure_count++;
    }

    // Future: Could implement adaptive strategies here
    // e.g., disable anchors with too many consecutive failures
}

void twr_scheduler_set_strategy(twr_scheduler_strategy_e new_strategy)
{
    strategy = new_strategy;
    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Strategy set to %d", new_strategy);
}

void twr_scheduler_get_status(twr_scheduler_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->anchor_count   = anchor_count;
    status->current_index  = current_index;
    status->current_target = (anchor_count > 0) ? anchor_list[current_index].address : 0x0000;
    status->strategy       = strategy;
}

uint8_t twr_scheduler_get_anchor_count(void)
{
    return anchor_count;
}