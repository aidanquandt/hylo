/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_manager.h"
#include "../twr/twr_engine/twr_types.h"            // For twr_result_t
#include "../twr/twr_roles/initiator/initiator.h" // Direct initiator interface
#include "twr_scheduler/twr_scheduler.h"
#include "backoff.h"
#include "error_handler.h"
#include "module.h"
#include "state_machine.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define RANGING_RATE_HZ 100                           // Target ranging rate (Hz)
#define RANGING_PERIOD_TICKS (1000 / RANGING_RATE_HZ) // Period in 1kHz ticks

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    STATE_IDLE = 0,    // Not actively ranging
    STATE_RANGING,     // Normal ranging operation
    STATE_BACKING_OFF, // Waiting after consecutive failures
    STATE_FAULTED      // Unrecoverable error
} twr_manager_state_e;

typedef struct
{
    uint32_t success_count;        // Total successful ranges
    uint32_t failure_count;        // Total failed ranges
    uint32_t consecutive_failures; // Track consecutive failures for backoff
    uint32_t backoff_delay_ms;     // Current backoff delay
    uint16_t rate_prescaler;       // Rate limiting counter
    bool ranging_in_progress;      // Track if ranging attempt is active
} twr_manager_ctx_t;

typedef struct
{
    bool needs_backoff; // Flag: fresh failure requires backoff
} twr_manager_sm_inputs_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void twr_manager_state_idle_process(void);
STATIC void twr_manager_state_idle_on_entry(uint16_t prev_state);
STATIC void twr_manager_state_ranging_process(void);
STATIC void twr_manager_state_ranging_on_entry(uint16_t prev_state);
STATIC void twr_manager_state_backing_off_process(void);
STATIC void twr_manager_state_backing_off_on_entry(uint16_t prev_state);
STATIC void twr_manager_state_faulted_process(void);
STATIC void twr_manager_state_faulted_on_entry(uint16_t prev_state);
STATIC uint16_t twr_manager_transition_logic(uint16_t current_state, uint32_t state_timer);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void twr_manager_init(void);
STATIC void twr_manager_process_1kHz(void);

extern const module_S twr_manager_module;

const module_S twr_manager_module = {
    .module_name         = "twr_manager",
    .module_init         = twr_manager_init,
    .module_process_1kHz = twr_manager_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC twr_manager_ctx_t ctx                     = {0};
STATIC twr_manager_sm_inputs_t sm_inputs         = {0};
STATIC state_machine_s twr_manager_state_machine = {0};
STATIC const backoff_config_t backoff_config     = {
        .min_delay_ms = 10, .max_delay_ms = 1000, .base_multiplier = 3, .use_jitter = false};
STATIC const state_s states[] = {
    [STATE_IDLE]        = {.process = twr_manager_state_idle_process,
                           .onEntry = twr_manager_state_idle_on_entry,
                           .onExit  = NULL},
    [STATE_RANGING]     = {.process = twr_manager_state_ranging_process,
                           .onEntry = twr_manager_state_ranging_on_entry,
                           .onExit  = NULL},
    [STATE_BACKING_OFF] = {.process = twr_manager_state_backing_off_process,
                           .onEntry = twr_manager_state_backing_off_on_entry,
                           .onExit  = NULL},
    [STATE_FAULTED]     = {.process = twr_manager_state_faulted_process,
                           .onEntry = twr_manager_state_faulted_on_entry,
                           .onExit  = NULL}};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void twr_manager_init(void)
{
    twr_scheduler_init();

    ctx.success_count        = 0;
    ctx.failure_count        = 0;
    ctx.consecutive_failures = 0;
    ctx.backoff_delay_ms     = 0;
    ctx.rate_prescaler       = 0;
    ctx.ranging_in_progress  = false;

    sm_inputs.needs_backoff = false;

    // Initialize state machine
    twr_manager_state_machine.curr_state      = STATE_IDLE;
    twr_manager_state_machine.prev_state      = STATE_IDLE;
    twr_manager_state_machine.next_state      = STATE_IDLE;
    twr_manager_state_machine.timer           = 0;
    twr_manager_state_machine.transitionLogic = twr_manager_transition_logic;
    twr_manager_state_machine.states          = states;
}

STATIC void twr_manager_process_1kHz(void)
{
    state_machine_periodic(&twr_manager_state_machine);
}

STATIC uint16_t twr_manager_transition_logic(uint16_t current_state, uint32_t state_timer)
{
    (void)state_timer; // Unused for most states
    uint16_t next_state = current_state;

    switch (current_state)
    {
        case STATE_IDLE:
            // Stay in IDLE, transitions happen via twr_manager_start()
            break;

        case STATE_RANGING:
            // Transition to BACKING_OFF only on fresh failure
            if (sm_inputs.needs_backoff)
            {
                sm_inputs.needs_backoff = false;
                next_state              = STATE_BACKING_OFF;
            }
            break;

        case STATE_BACKING_OFF:
            // Transition back to RANGING after backoff timer expires
            if (state_timer >= ctx.backoff_delay_ms)
            {
                next_state = STATE_RANGING;
            }
            break;

        case STATE_FAULTED:
            // Stay faulted until manual intervention
            break;

        default:
            next_state = STATE_IDLE;
            break;
    }

    return next_state;
}

STATIC void twr_manager_state_idle_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    error_handler_log(ERROR_SEVERITY_INFO, "twr_mgr", "Entering IDLE state");
}

STATIC void twr_manager_state_idle_process(void)
{
    // Do nothing - waiting for twr_manager_start()
}

STATIC void twr_manager_state_ranging_on_entry(uint16_t prev_state)
{
    error_handler_log(ERROR_SEVERITY_INFO, "twr_mgr", "Entering RANGING state (from state %d)",
                      prev_state);

    // Only reset consecutive failures when starting fresh from IDLE
    // NOT when coming back from backoff - we need to keep the count to increase delays
    if (prev_state == STATE_IDLE)
    {
        ctx.consecutive_failures = 0;
        ctx.backoff_delay_ms     = 0;
    }

    ctx.rate_prescaler      = 0;
    ctx.ranging_in_progress = false;
}

STATIC void twr_manager_state_ranging_process(void)
{
    // Check if ranging completed
    bool is_ranging = initiator_is_ranging();
    if (ctx.ranging_in_progress && !is_ranging)
    {
        // Ranging just completed - process result ONCE
        ctx.ranging_in_progress = false;

        twr_result_t result;
        bool ranging_succeeded = initiator_get_last_result(&result);

        // Get the target we just ranged with
        uint16_t ranged_target = twr_scheduler_get_current_target();

        if (ranging_succeeded)
        {
            // Success - reset backoff and advance to next target
            ctx.success_count++;
            ctx.consecutive_failures = 0;
            ctx.backoff_delay_ms     = 0;
            sm_inputs.needs_backoff  = false;

            // Report success to scheduler and advance
            twr_scheduler_report_result(ranged_target, true);
            twr_scheduler_get_next_target(); // Advance to next
        }
        else
        {
            // Failure - increment counter and trigger backoff
            ctx.failure_count++;
            ctx.consecutive_failures++;

            // Calculate backoff delay for next state
            ctx.backoff_delay_ms    = backoff_calculate(&backoff_config, ctx.consecutive_failures);
            sm_inputs.needs_backoff = true; // Signal transition to BACKING_OFF

            // Report failure to scheduler and advance
            twr_scheduler_report_result(ranged_target, false);
            twr_scheduler_get_next_target(); // Advance to next

            error_handler_log(ERROR_SEVERITY_WARNING, "twr_mgr",
                              "Ranging failed (consecutive: %lu)",
                              (unsigned long)ctx.consecutive_failures);
        }
    }

    // Rate limiting: only attempt ranging at configured rate
    ctx.rate_prescaler++;
    if (ctx.rate_prescaler >= RANGING_PERIOD_TICKS)
    {
        ctx.rate_prescaler = 0;

        // Only start new ranging if not already in progress
        if (!is_ranging && !ctx.ranging_in_progress)
        {
            uint16_t target_peer = twr_scheduler_get_current_target();
            if (target_peer != 0x0000 && initiator_start_ranging(target_peer))
            {
                ctx.ranging_in_progress = true;
            }
        }
    }
}

STATIC void twr_manager_state_backing_off_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    error_handler_log(ERROR_SEVERITY_WARNING, "twr_mgr",
                      "Entering BACKOFF state - waiting %lu ms (failures: %lu)",
                      (unsigned long)ctx.backoff_delay_ms, (unsigned long)ctx.consecutive_failures);
}

STATIC void twr_manager_state_backing_off_process(void)
{
    // Just wait for timer to expire - transition logic handles exit
}

STATIC void twr_manager_state_faulted_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    error_handler_log(ERROR_SEVERITY_ERROR, "twr_mgr",
                      "Entering FAULTED state - manual recovery required");
}

STATIC void twr_manager_state_faulted_process(void)
{
    // Faulted - do nothing, requires manual recovery via stop/start
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool twr_manager_start(void)
{
    // Validate that targets are configured
    if (twr_scheduler_get_target_count() == 0)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twrmgr",
                          "Cannot start: No targets configured. Use twrmgr.add.target or "
                          "twrmgr.set.targets first");
        return false;
    }

    // Initialize and start initiator mode
    initiator_init();
    if (!initiator_start())
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr_mgr", "Failed to start initiator");
        state_machine_force_transition(&twr_manager_state_machine, STATE_FAULTED);
        return false;
    }

    // Reset counters and transition to RANGING state
    ctx.success_count        = 0;
    ctx.failure_count        = 0;
    ctx.consecutive_failures = 0;
    ctx.backoff_delay_ms     = 0;
    ctx.rate_prescaler       = 0;
    ctx.ranging_in_progress  = false;
    sm_inputs.needs_backoff  = false;

    state_machine_force_transition(&twr_manager_state_machine, STATE_RANGING);
    return true;
}

void twr_manager_stop(void)
{
    // Cancel any ongoing ranging and stop initiator
    if (ctx.ranging_in_progress)
    {
        initiator_cancel_ranging();
    }
    initiator_stop();

    // Transition to IDLE
    state_machine_force_transition(&twr_manager_state_machine, STATE_IDLE);
}

bool twr_manager_is_active(void)
{
    return (twr_manager_state_machine.curr_state == STATE_RANGING ||
            twr_manager_state_machine.curr_state == STATE_BACKING_OFF);
}

uint32_t twr_manager_get_success_count(void)
{
    return ctx.success_count;
}

uint32_t twr_manager_get_failure_count(void)
{
    return ctx.failure_count;
}
