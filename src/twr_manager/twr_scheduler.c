/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_scheduler.h"
#include "FreeRTOS.h"
#include "backoff.h"
#include "common.h"
#include "protocol_tx.h"
#include "task.h"
#include "protocol.pb.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TWR_TARGET_BACKOFF_MIN_MS (10U)   // Minimum backoff delay (2 ranging cycles at 200Hz)
#define TWR_TARGET_BACKOFF_MAX_MS (2000U) // Maximum backoff delay
#define TWR_TARGET_WARN_THRESHOLD (2U)    // Log warning only after this many consecutive failures

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

    uint32_t now = xTaskGetTickCount();

    // Find next enabled target that's not in backoff
    uint8_t attempts = 0;
    while (attempts < target_count)
    {
        // Advance to next index
        current_index = (current_index + 1) % target_count;

        // Check if this target is enabled and not in backoff
        if (target_list[current_index].enabled &&
            (now >= target_list[current_index].backoff_until_ms))
        {
            return target_list[current_index].address;
        }

        attempts++;
    }

    // All targets disabled or in backoff - return 0x0000
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
        {
            // Future implementation
            TwrSchedulerStrategyNotImplementedEvent ev = TwrSchedulerStrategyNotImplementedEvent_init_zero;
            protocol_tx_TwrSchedulerStrategyNotImplementedEvent(&ev);
            next_address = get_next_round_robin(); // Fallback
            break;
        }

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

    uint32_t now = xTaskGetTickCount();

    // Return current target if enabled and not in backoff
    if (target_list[current_index].enabled && (now >= target_list[current_index].backoff_until_ms))
    {
        return target_list[current_index].address;
    }

    // If current is disabled or backed off, get next valid one
    return twr_scheduler_get_next_target();
}

bool twr_scheduler_add_target(uint16_t address)
{
    // Check if list is full
    if (target_count >= TWR_SCHEDULER_MAX_TARGETS)
    {
        TwrSchedulerTargetListFullEvent ev = TwrSchedulerTargetListFullEvent_init_zero;
        protocol_tx_TwrSchedulerTargetListFullEvent(&ev);
        return false;
    }

    // Check if already exists
    if (find_target_index(address) >= 0)
    {
        TwrSchedulerTargetAlreadyExistsEvent ev = TwrSchedulerTargetAlreadyExistsEvent_init_zero;
        ev.address = address;
        protocol_tx_TwrSchedulerTargetAlreadyExistsEvent(&ev);
        return false;
    }

    // Add new target
    target_list[target_count].address              = address;
    target_list[target_count].priority             = 0;
    target_list[target_count].enabled              = true;
    target_list[target_count].success_count        = 0;
    target_list[target_count].failure_count        = 0;
    target_list[target_count].consecutive_failures = 0;
    target_list[target_count].backoff_until_ms     = 0;
    target_count++;

    TwrSchedulerAddedTargetEvent ev = TwrSchedulerAddedTargetEvent_init_zero;
    ev.address      = address;
    ev.target_count = target_count;
    protocol_tx_TwrSchedulerAddedTargetEvent(&ev);

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

    TwrSchedulerRemovedTargetEvent ev = TwrSchedulerRemovedTargetEvent_init_zero;
    ev.address   = address;
    ev.remaining = target_count;
    protocol_tx_TwrSchedulerRemovedTargetEvent(&ev);

    return true;
}

bool twr_scheduler_set_targets(const uint16_t* addresses, uint8_t count)
{
    if (addresses == NULL)
    {
        TwrSchedulerNullAddressesPointerEvent ev = TwrSchedulerNullAddressesPointerEvent_init_zero;
        protocol_tx_TwrSchedulerNullAddressesPointerEvent(&ev);
        return false;
    }

    if (count > TWR_SCHEDULER_MAX_TARGETS)
    {
        TwrSchedulerTargetCountExceedsMaximumEvent ev = TwrSchedulerTargetCountExceedsMaximumEvent_init_zero;
        ev.count   = count;
        ev.maximum = TWR_SCHEDULER_MAX_TARGETS;
        protocol_tx_TwrSchedulerTargetCountExceedsMaximumEvent(&ev);
        return false;
    }

    // Clear existing list
    twr_scheduler_clear_all();

    // Add all targets
    for (uint8_t i = 0; i < count; i++)
    {
        target_list[i].address              = addresses[i];
        target_list[i].priority             = 0;
        target_list[i].enabled              = true;
        target_list[i].success_count        = 0;
        target_list[i].failure_count        = 0;
        target_list[i].consecutive_failures = 0;
        target_list[i].backoff_until_ms     = 0;
    }

    target_count  = count;
    current_index = 0;

    TwrSchedulerSetTargetsEvent ev = TwrSchedulerSetTargetsEvent_init_zero;
    ev.count = count;
    protocol_tx_TwrSchedulerSetTargetsEvent(&ev);

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

    TwrSchedulerClearedAllTargetsEvent ev = TwrSchedulerClearedAllTargetsEvent_init_zero;
    protocol_tx_TwrSchedulerClearedAllTargetsEvent(&ev);
}

bool twr_scheduler_set_target_enabled(uint16_t address, bool enabled)
{
    int8_t index = find_target_index(address);
    if (index < 0)
    {
        return false;
    }

    target_list[index].enabled = enabled;

    TwrSchedulerTargetEnabledDisabledEvent ev = TwrSchedulerTargetEnabledDisabledEvent_init_zero;
    ev.address = address;
    ev.enabled = enabled;
    protocol_tx_TwrSchedulerTargetEnabledDisabledEvent(&ev);

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
        target_list[index].consecutive_failures = 0;
        target_list[index].backoff_until_ms     = 0; // Clear any backoff
    }
    else
    {
        target_list[index].failure_count++;
        target_list[index].consecutive_failures++;

        // Calculate exponential backoff for this target
        uint32_t backoff_delay =
            backoff_calculate_simple(target_list[index].consecutive_failures,
                                     TWR_TARGET_BACKOFF_MIN_MS, TWR_TARGET_BACKOFF_MAX_MS, true);

        uint32_t now                        = xTaskGetTickCount();
        target_list[index].backoff_until_ms = now + backoff_delay;

        if (target_list[index].consecutive_failures > TWR_TARGET_WARN_THRESHOLD)
        {
            TwrSchedulerTargetBackingOffEvent ev = TwrSchedulerTargetBackingOffEvent_init_zero;
            ev.address    = address;
            ev.backoff_ms = (uint32_t)backoff_delay;
            ev.failures   = target_list[index].consecutive_failures;
            protocol_tx_TwrSchedulerTargetBackingOffEvent(&ev);
        }
    }
}

void twr_scheduler_set_strategy(twr_scheduler_strategy_e new_strategy)
{
    strategy = new_strategy;
    TwrSchedulerStrategySetEvent ev = TwrSchedulerStrategySetEvent_init_zero;
    ev.strategy = (int32_t)new_strategy;
    protocol_tx_TwrSchedulerStrategySetEvent(&ev);
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
