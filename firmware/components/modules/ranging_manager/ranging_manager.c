/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "ranging_manager.h"
#include "error_handler.h"
#include "module.h"
#include "ranging.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void ranging_manager_init(void);
STATIC void ranging_manager_process_1kHz(void);

extern const module_S ranging_manager_module;

const module_S ranging_manager_module = {
    .module_name         = "ranging_manager",
    .module_init         = ranging_manager_init,
    .module_process_1kHz = ranging_manager_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/
#define TARGET_ANCHOR_ADDRESS 0x0001                  // Default anchor to range with
#define RANGING_RATE_HZ 100                           // Target ranging rate (Hz)
#define RANGING_PERIOD_TICKS (1000 / RANGING_RATE_HZ) // Period in 1kHz ticks

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    bool active;             // Ranging manager is active
    bool initialized;        // Module initialized
    uint16_t target_anchor;  // Current target anchor address
    uint32_t success_count;  // Total successful ranges
    uint32_t failure_count;  // Total failed ranges
    uint16_t rate_prescaler; // Prescaler counter for rate limiting
} ranging_manager_ctx_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC ranging_manager_ctx_t ctx = {0};

/*---------------------------------------------------------------------------
 * Module Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void ranging_manager_init(void)
{
    ctx.active         = false;
    ctx.initialized    = true;
    ctx.target_anchor  = TARGET_ANCHOR_ADDRESS;
    ctx.success_count  = 0;
    ctx.failure_count  = 0;
    ctx.rate_prescaler = 0;
}

STATIC void ranging_manager_process_1kHz(void)
{
    if (!ctx.active || !ctx.initialized)
    {
        return;
    }

    // Rate limiting: only attempt ranging at configured rate
    ctx.rate_prescaler++;
    if (ctx.rate_prescaler >= RANGING_PERIOD_TICKS)
    {
        ctx.rate_prescaler = 0;

        // Only start new ranging if previous one completed
        if (!ranging_tag_is_active())
        {
            // Check if last result was successful or failed
            float distance_m;
            if (ranging_tag_get_result(&distance_m, NULL))
            {
                ctx.success_count++;
            }
            else if (ctx.success_count + ctx.failure_count > 0)
            {
                // Only count as failure if we've attempted at least once
                ctx.failure_count++;
            }

            // Start new ranging attempt
            if (!ranging_tag_start(ctx.target_anchor))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "ranging_mgr",
                                  "Failed to start ranging with anchor 0x%04X", ctx.target_anchor);
            }
        }
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool ranging_manager_start(void)
{
    if (!ctx.initialized)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "ranging_mgr", "Not initialized");
        return false;
    }

    // Set ranging mode to TAG
    if (!ranging_set_mode(RANGING_MODE_TAG))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "ranging_mgr", "Failed to set TAG mode");
        return false;
    }

    ctx.active         = true;
    ctx.rate_prescaler = 0;

    return true;
}

void ranging_manager_stop(void)
{
    if (!ctx.initialized)
    {
        return;
    }

    ctx.active = false;

    // Cancel any ongoing ranging
    ranging_tag_cancel();

    // Disable ranging mode
    ranging_set_mode(RANGING_MODE_DISABLED);
}

bool ranging_manager_is_active(void)
{
    return ctx.active;
}

uint32_t ranging_manager_get_success_count(void)
{
    return ctx.success_count;
}

uint32_t ranging_manager_get_failure_count(void)
{
    return ctx.failure_count;
}
