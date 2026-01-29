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
STATIC twr_target_entry_t target_list[TWR_SCHEDULER_MAX_TARGETS];
STATIC uint8_t target_count;
STATIC uint8_t current_index;
STATIC twr_scheduler_strategy_e strategy;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t find_target_index(uint16_t address);
STATIC uint16_t get_next_round_robin(void);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC int8_t find_target_index(uint16_t address)
{
    for (uint8_t i = 0; i < target_count; i++)
    {
        if (target_list[i].address == address)
        {
            return (int8_t)i;
        }
    }
    return -1; // Not found
}

STATIC uint16_t get_next_round_robin(void)
{
    if (target_count == 0)
    {
        return 0x0000;
    }

    // Find next enabled target
    uint8_t attempts = 0;
    while (attempts < target_count)
    {
        // Advance to next index
        current_index = (current_index + 1) % target_count;

        // Check if this target is enabled
        if (target_list[current_index].enabled)
        {
            return target_list[current_index].address;
        }

        attempts++;
    }

    // All targets disabled - return 0x0000
    return 0x0000;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void twr_scheduler_init(void)
{
    memset(target_list, 0, sizeof(target_list));
    target_count  = 0;
    current_index = 0;
    strategy      = TWR_SCHED_STRATEGY_ROUND_ROBIN;
}

uint16_t twr_scheduler_get_next_target(void)
{
    if (target_count == 0)
    {
        return 0x0000; // No targets configured
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
    if (target_count == 0)
    {
        return 0x0000;
    }

    // Bounds check
    if (current_index >= target_count)
    {
        current_index = 0; // Reset to start
    }

    // Return current target without advancing
    if (target_list[current_index].enabled)
    {
        return target_list[current_index].address;
    }

    // If current is disabled, get next valid one
    return twr_scheduler_get_next_target();
}

bool twr_scheduler_add_target(uint16_t address)
{
    // Check if list is full
    if (target_count >= TWR_SCHEDULER_MAX_TARGETS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "Target list full");
        return false;
    }

    // Check if already exists
    if (find_target_index(address) >= 0)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "twr_sched", "Target 0x%04X already exists",
                          address);
        return false;
    }

    // Add new target
    target_list[target_count].address       = address;
    target_list[target_count].priority      = 0;
    target_list[target_count].enabled       = true;
    target_list[target_count].success_count = 0;
    target_list[target_count].failure_count = 0;
    target_count++;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Added target 0x%04X (total: %d)", address,
                      target_count);

    return true;
}

bool twr_scheduler_remove_target(uint16_t address)
{
    int8_t index = find_target_index(address);
    if (index < 0)
    {
        return false; // Not found
    }

    // Shift remaining targets down
    for (uint8_t i = index; i < target_count - 1; i++)
    {
        target_list[i] = target_list[i + 1];
    }

    target_count--;

    // Clear last entry
    memset(&target_list[target_count], 0, sizeof(twr_target_entry_t));

    // Adjust current index if needed
    if (current_index >= target_count && target_count > 0)
    {
        current_index = 0; // Wrap around
    }

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Removed target 0x%04X (remaining: %d)",
                      address, target_count);

    return true;
}

bool twr_scheduler_set_targets(const uint16_t* addresses, uint8_t count)
{
    if (addresses == NULL)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "NULL addresses pointer");
        return false;
    }

    if (count > TWR_SCHEDULER_MAX_TARGETS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_sched", "Target count %d exceeds maximum %d",
                          count, TWR_SCHEDULER_MAX_TARGETS);
        return false;
    }

    // Clear existing list
    twr_scheduler_clear_all();

    // Add all targets
    for (uint8_t i = 0; i < count; i++)
    {
        target_list[i].address       = addresses[i];
        target_list[i].priority      = 0;
        target_list[i].enabled       = true;
        target_list[i].success_count = 0;
        target_list[i].failure_count = 0;
    }

    target_count  = count;
    current_index = 0;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Set %d targets", count);

    return true;
}

void twr_scheduler_clear_all(void)
{
    // Only clear entries that were actually used
    if (target_count > 0)
    {
        memset(target_list, 0, target_count * sizeof(twr_target_entry_t));
    }
    target_count  = 0;
    current_index = 0;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Cleared all targets");
}

bool twr_scheduler_set_target_enabled(uint16_t address, bool enabled)
{
    int8_t index = find_target_index(address);
    if (index < 0)
    {
        return false;
    }

    target_list[index].enabled = enabled;

    error_handler_log(ERROR_SEVERITY_INFO, "twr_sched", "Target 0x%04X %s", address,
                      enabled ? "enabled" : "disabled");

    return true;
}

void twr_scheduler_report_result(uint16_t address, bool success)
{
    int8_t index = find_target_index(address);
    if (index < 0)
    {
        return; // Target not found - silently ignore
    }

    if (success)
    {
        target_list[index].success_count++;
    }
    else
    {
        target_list[index].failure_count++;
    }

    // Future: Could implement adaptive strategies here
    // e.g., disable targets with too many consecutive failures
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

    status->target_count   = target_count;
    status->current_index  = current_index;
    status->current_target = (target_count > 0) ? target_list[current_index].address : 0x0000;
    status->strategy       = strategy;
}

uint8_t twr_scheduler_get_target_count(void)
{
    return target_count;
}
