/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_manager.h"
#include "twr_types.h"     // For twr_result_t (from twr)
#include "initiator.h"     // Direct initiator interface (from twr)
#include "FreeRTOS.h"
#include "backoff.h"
#include "common.h"
#include "error_handler.h"
#include "feature_config.h"
#include "module.h"
#include "sensor_fusion.h"
#include "state_machine.h"
#include "task.h"
#include "twr_scheduler.h"


/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define RANGING_RATE_HZ_DEFAULT 200 // Default ranging rate (Hz)
#define RANGING_RATE_HZ_MIN 1       // Minimum ranging rate (Hz)
#define RANGING_RATE_HZ_MAX 200     // Maximum ranging rate (Hz)
#define INVALID_TARGET_ADDR 0x0000  // Invalid/unset target address

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uint16_t ranging_rate_hz      = RANGING_RATE_HZ_DEFAULT;
STATIC uint16_t ranging_period_ticks = (1000 / RANGING_RATE_HZ_DEFAULT);

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    STATE_IDLE = 0, // Not actively ranging
    STATE_RANGING,  // Normal ranging operation
    STATE_FAULTED   // Unrecoverable error
} twr_manager_state_e;

typedef struct
{
    uint32_t success_count;   // Total successful ranges
    uint32_t failure_count;   // Total failed ranges
    uint16_t rate_prescaler;  // Rate limiting counter
    bool ranging_in_progress; // Track if ranging attempt is active
} twr_manager_ctx_t;

typedef struct
{
    bool reserved; // Reserved for future use
} twr_manager_sm_inputs_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void twr_manager_state_idle_process(void);
STATIC void twr_manager_state_idle_on_entry(uint16_t prev_state);
STATIC void twr_manager_state_ranging_process(void);
STATIC void twr_manager_state_ranging_on_entry(uint16_t prev_state);
STATIC void twr_manager_state_faulted_on_entry(uint16_t prev_state);
STATIC uint16_t twr_manager_transition_logic(uint16_t current_state, uint32_t state_timer);
STATIC void twr_manager_push_to_sensor_fusion(const twr_result_t* result);
STATIC void twr_manager_handle_ranging_success(const twr_result_t* result, uint16_t target);
STATIC void twr_manager_handle_ranging_failure(uint16_t target);
STATIC void twr_manager_try_start_ranging(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void twr_manager_init(void);
STATIC void twr_manager_process_1kHz(void);

extern const module_S twr_manager_module;

const module_S twr_manager_module = {
    .module_name         = "twr_manager",
    .module_init         = twr_manager_init,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = twr_manager_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC twr_manager_ctx_t ctx                     = {0};
STATIC twr_manager_sm_inputs_t sm_inputs         = {0};
STATIC state_machine_s twr_manager_state_machine = {0};
STATIC const state_s states[]                    = {
    [STATE_IDLE]    = {.process = twr_manager_state_idle_process,
                                          .onEntry = twr_manager_state_idle_on_entry,
                                          .onExit  = NULL},
    [STATE_RANGING] = {.process = twr_manager_state_ranging_process,
                                          .onEntry = twr_manager_state_ranging_on_entry,
                                          .onExit  = NULL},
    [STATE_FAULTED] = {
                           .process = NULL, .onEntry = twr_manager_state_faulted_on_entry, .onExit = NULL}};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void twr_manager_push_to_sensor_fusion(const twr_result_t* result)
{
    if (result == NULL || !result->valid)
    {
        return;
    }

    sensor_event_t event = {.type         = SENSOR_EVENT_RANGING,
                            .timestamp_ms = result->timestamp_ms,
                            .sequence     = ctx.success_count,
                            .data.ranging = {.distance_m            = result->distance_m,
                                             .rssi_dbm              = result->rssi_dbm,
                                             .anchor_position       = result->anchor_position,
                                             .anchor_position_valid = result->anchor_position_valid,
                                             .anchor_addr           = result->remote_addr,
                                             .quality = 1.0f}}; // TODO: Derive quality metric

    sensor_fusion_status_e sf_status = sensor_fusion_push_event(&event);
    if (sf_status != SENSOR_FUSION_SUCCESS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "twr_mgr",
                          "Failed to push ranging event to sensor fusion: %d", sf_status);
    }
}

STATIC void twr_manager_handle_ranging_success(const twr_result_t* result, uint16_t target)
{
    // Update counters
    ctx.success_count++;

#if FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE
    // Log result with position if available
    if (result->anchor_position_valid)
    {
        error_handler_log(ERROR_SEVERITY_INFO, "twr_mgr",
                          "Range: %.2f m to 0x%04X at (%.2f, %.2f, %.2f)", result->distance_m,
                          result->remote_addr, result->anchor_position.x, result->anchor_position.y,
                          result->anchor_position.z);
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_INFO, "twr_mgr",
                          "Range: %.2f m to 0x%04X (position unknown)", result->distance_m,
                          result->remote_addr);
    }
#endif
    // Push to sensor fusion for localization
    twr_manager_push_to_sensor_fusion(result);

    // Report to scheduler (handles per-target success state)
    twr_scheduler_report_result(target, true);
    twr_scheduler_get_next_target();
}

STATIC void twr_manager_handle_ranging_failure(uint16_t target)
{
    // Update failure counter
    ctx.failure_count++;

    // Report to scheduler (handles per-target backoff)
    twr_scheduler_report_result(target, false);
    twr_scheduler_get_next_target();
}

STATIC void twr_manager_try_start_ranging(void)
{
    // Rate limiting: only attempt ranging at configured rate
    ctx.rate_prescaler++;
    if (ctx.rate_prescaler < ranging_period_ticks)
    {
        return;
    }

    ctx.rate_prescaler = 0;

    // Don't start if already ranging
    if (initiator_is_ranging() || ctx.ranging_in_progress)
    {
        return;
    }

    // Get target and validate
    uint16_t target_peer = twr_scheduler_get_current_target();
    if (target_peer == INVALID_TARGET_ADDR)
    {
        return;
    }

    // Attempt to start ranging
    if (initiator_start_ranging(target_peer))
    {
        ctx.ranging_in_progress = true;
    }
}

STATIC void twr_manager_init(void)
{
    twr_scheduler_init();

    ctx.success_count       = 0;
    ctx.failure_count       = 0;
    ctx.rate_prescaler      = 0;
    ctx.ranging_in_progress = false;

    sm_inputs.reserved = false;

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
    (void)state_timer; // Unused
    uint16_t next_state = current_state;

    switch (current_state)
    {
        case STATE_IDLE:
            // Stay in IDLE, transitions happen via twr_manager_start()
            break;

        case STATE_RANGING:
            // Stay in RANGING - scheduler handles per-target backoff
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

    ctx.rate_prescaler      = 0;
    ctx.ranging_in_progress = false;
}

STATIC void twr_manager_state_ranging_process(void)
{
    // Check if ranging just completed
    if (ctx.ranging_in_progress && !initiator_is_ranging())
    {
        ctx.ranging_in_progress = false;

        twr_result_t result;
        bool success    = initiator_get_last_result(&result);
        uint16_t target = twr_scheduler_get_current_target();

        if (success)
        {
            twr_manager_handle_ranging_success(&result, target);
        }
        else
        {
            twr_manager_handle_ranging_failure(target);
        }
    }

    // Periodically attempt to start new ranging
    twr_manager_try_start_ranging();
}

STATIC void twr_manager_state_faulted_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    error_handler_log(ERROR_SEVERITY_ERROR, "twr_mgr",
                      "Entering FAULTED state - manual recovery required");
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
    ctx.success_count       = 0;
    ctx.failure_count       = 0;
    ctx.rate_prescaler      = 0;
    ctx.ranging_in_progress = false;
    sm_inputs.reserved      = false;

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
    return (twr_manager_state_machine.curr_state == STATE_RANGING);
}

uint32_t twr_manager_get_success_count(void)
{
    return ctx.success_count;
}

uint32_t twr_manager_get_failure_count(void)
{
    return ctx.failure_count;
}

bool twr_manager_set_ranging_rate(uint16_t rate_hz)
{
    if (rate_hz < RANGING_RATE_HZ_MIN || rate_hz > RANGING_RATE_HZ_MAX)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twrmgr",
                          "Invalid ranging rate %u Hz (valid range: %u-%u Hz)", rate_hz,
                          RANGING_RATE_HZ_MIN, RANGING_RATE_HZ_MAX);
        return false;
    }

    ranging_rate_hz      = rate_hz;
    ranging_period_ticks = (1000 / rate_hz);

    error_handler_log(ERROR_SEVERITY_INFO, "twrmgr", "Ranging rate set to %u Hz (period: %u ms)",
                      rate_hz, ranging_period_ticks);
    return true;
}

uint16_t twr_manager_get_ranging_rate(void)
{
    return ranging_rate_hz;
}
