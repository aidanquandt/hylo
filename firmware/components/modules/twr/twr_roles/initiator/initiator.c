/*---------------------------------------------------------------------------
 * @file    initiator.c
 * @brief   TWR Initiator implementation
 * @details Initiates ranging transactions using unified TWR state machine
 *---------------------------------------------------------------------------*/

#include "initiator.h"
#include "../twr.h"
#include "../twr_engine/twr_algorithm.h"
#include "../twr_engine/twr_state_machine.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "feature_config.h"
#include "stopwatch.h"
#include "task.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"


/*---------------------------------------------------------------------------

 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool initiator_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx);
STATIC void initiator_handle_message_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void initiator_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void initiator_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void initiator_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void initiator_process_result_impl(twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
twr_context_t initiator_twr_ctx; // Made non-static for external access
STATIC initiator_status_t initiator_status = {0};

STATIC const twr_callbacks_t initiator_callbacks = {
    .send_message       = initiator_send_message_impl,
    .handle_message     = initiator_handle_message_impl,
    .handle_tx_complete = initiator_handle_tx_complete_impl,
    .handle_timeout     = initiator_handle_timeout_impl,
    .handle_fault       = initiator_handle_fault_impl,
    .process_result     = initiator_process_result_impl};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool initiator_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx)
{
    uwb_send_result_t result;

    switch (msg_type)
    {
        case TWR_MSG_POLL:
        {
            protocol_twr_poll_msg_t poll = {.header = {.protocol_type = PROTOCOL_TYPE_TWR,
                                                       .msg_type      = TWR_MSG_POLL,
                                                       .sequence      = ctx->sequence}};

            result = uwb_send_message((uint8_t*)&poll, sizeof(poll), ctx->peer_address);
            break;
        }

        case TWR_MSG_FINAL:
        {
            protocol_twr_final_msg_t final = {.header = {.protocol_type = PROTOCOL_TYPE_TWR,
                                                         .msg_type      = TWR_MSG_FINAL,
                                                         .sequence      = ctx->sequence}};

            // Include response RX timestamp
            twr_u64_to_timestamp(ctx->timestamps[1].timestamp_dtu, final.resp_rx_ts);
            memset(final.final_tx_ts, 0, sizeof(final.final_tx_ts)); // Filled by TX callback

            result = uwb_send_message((uint8_t*)&final, sizeof(final), ctx->peer_address);
            break;
        }

        default:
            return false;
    }

    if (result.success)
    {
        ctx->pending_tx_id  = result.message_id;
        ctx->pending_tx_msg = msg_type; // Track what message we're sending
        return true;
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "initiator", "Failed to send message type %d",
                          msg_type);
        return false;
    }
}

STATIC void initiator_handle_message_impl(const twr_event_t* event, twr_context_t* ctx)
{
    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;
    twr_msg_type_e msg_type         = (twr_msg_type_e)header->msg_type;

    switch (msg_type)
    {
        case TWR_MSG_RESPONSE:
        {
            if (event->rx.length < sizeof(protocol_twr_response_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "initiator", "Response too short");
                return;
            }

            const protocol_twr_response_msg_t* resp =
                (const protocol_twr_response_msg_t*)event->rx.data;

            // Store response RX timestamp
            ctx->timestamps[1].timestamp_dtu = event->rx.rx_timestamp;
            ctx->timestamps[1].local_time_ms = xTaskGetTickCount();

            // Store responder's poll RX timestamp
            ctx->remote_timestamps[0] = twr_timestamp_to_u64(resp->poll_rx_ts);

            // Extract anchor position
            ctx->remote_anchor_position = resp->anchor_position;
            // Position is valid if any coordinate is non-zero
            ctx->remote_anchor_position_valid =
                (resp->anchor_position.x != 0.0f || resp->anchor_position.y != 0.0f ||
                 resp->anchor_position.z != 0.0f);

            // Prepare for final
            ctx->expected_msg = TWR_MSG_FINAL_ACK;

            // Send final
            if (ctx->callbacks->send_message(TWR_MSG_FINAL, ctx))
            {
                twr_state_machine_transition_to(ctx, TWR_STATE_SENDING);
            }
            else
            {
                ctx->fault_code         = TWR_FAULT_SEND_FAILED;
                twr_event_t fault_event = {.type  = TWR_EVENT_FAULT,
                                           .fault = {.fault_code  = TWR_FAULT_SEND_FAILED,
                                                     .description = "Final send failed"}};
                ctx->callbacks->handle_fault(&fault_event, ctx);
            }
            break;
        }

        case TWR_MSG_FINAL_ACK:
        {
            if (event->rx.length < sizeof(protocol_twr_final_ack_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "initiator", "Final ACK too short");
                return;
            }

            const protocol_twr_final_ack_msg_t* ack =
                (const protocol_twr_final_ack_msg_t*)event->rx.data;

            // Store responder's timestamps
            ctx->remote_timestamps[1] = twr_timestamp_to_u64(ack->resp_tx_ts);  // Response TX
            ctx->remote_timestamps[2] = twr_timestamp_to_u64(ack->final_rx_ts); // Final RX

            // Extract anchor position (redundant, but provides fallback)
            // Only update if we didn't get valid position from RESPONSE
            if (!ctx->remote_anchor_position_valid)
            {
                ctx->remote_anchor_position = ack->anchor_position;
                ctx->remote_anchor_position_valid =
                    (ack->anchor_position.x != 0.0f || ack->anchor_position.y != 0.0f ||
                     ack->anchor_position.z != 0.0f);
            }

            // Process the result
            twr_state_machine_transition_to(ctx, TWR_STATE_PROCESSING);
            break;
        }

        default:
            error_handler_log(ERROR_SEVERITY_WARNING, "initiator", "Unexpected message type %d",
                              msg_type);
            break;
    }
}

STATIC void initiator_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx)
{
    twr_msg_type_e msg_type = ctx->pending_tx_msg; // Use tracked message type

    // Store TX timestamp
    if (msg_type == TWR_MSG_POLL)
    {
        ctx->timestamps[0].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[0].local_time_ms = xTaskGetTickCount();

        // Wait for response
        ctx->expected_msg = TWR_MSG_RESPONSE;
        twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
    }
    else if (msg_type == TWR_MSG_FINAL)
    {
        ctx->timestamps[2].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[2].local_time_ms = xTaskGetTickCount();

        // Wait for final ACK
        ctx->expected_msg = TWR_MSG_FINAL_ACK;
        twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
    }

    ctx->pending_tx_id = 0; // Clear pending ID
}

STATIC void initiator_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx)
{
    (void)event; // Unused parameter

    // No retries at state machine level - let scheduler/manager handle retry logic
    ctx->failed_transactions++;
    ctx->last_result.valid = false; // Invalidate cached result on failure
    twr_state_machine_transition_to(ctx, TWR_STATE_IDLE);
}

STATIC void initiator_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_ERROR, "initiator", "Fault: %d - %s", event->fault.fault_code,
                      event->fault.description);

    ctx->fault_code = event->fault.fault_code;
    ctx->failed_transactions++;

    twr_state_machine_transition_to(ctx, TWR_STATE_IDLE);
}

STATIC void initiator_process_result_impl(twr_context_t* ctx)
{
    twr_result_t result = {0};

    // Calculate distance using DS-TWR with 6 timestamps
    twr_status_e status =
        twr_calculate_ds_twr(ctx->timestamps[0].timestamp_dtu, // Poll TX (initiator)
                             ctx->remote_timestamps[0],        // Poll RX (responder)
                             ctx->remote_timestamps[1],        // Response TX (responder)
                             ctx->timestamps[1].timestamp_dtu, // Response RX (initiator)
                             ctx->timestamps[2].timestamp_dtu, // Final TX (initiator)
                             ctx->remote_timestamps[2],        // Final RX (responder)
                             &result);

    if (status == TWR_SUCCESS && result.valid)
    {
        stopwatch_stop(0);

        result.remote_addr  = ctx->peer_address;
        result.timestamp_ms = xTaskGetTickCount();

        // Include anchor position from responder
        result.anchor_position       = ctx->remote_anchor_position;
        result.anchor_position_valid = ctx->remote_anchor_position_valid;

        ctx->last_result = result;
        ctx->successful_transactions++;
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "initiator", "Distance calculation failed: %d",
                          status);
        ctx->failed_transactions++;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void initiator_init(void)
{
    twr_state_machine_init(&initiator_twr_ctx, TWR_ROLE_INITIATOR, &initiator_callbacks,
                           &initiator_status);
    memset(&initiator_status, 0, sizeof(initiator_status));
}

bool initiator_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    // No registration needed - protocol handler dispatches based on mode
    return true;
}

void initiator_stop(void)
{
    twr_state_machine_stop(&initiator_twr_ctx);
}

bool initiator_start_ranging(uint16_t responder_addr)
{
    if (!uwb_is_ready())
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "initiator", "UWB not ready");
        return false;
    }

    stopwatch_start(0);

    if (twr_state_machine_start(&initiator_twr_ctx, responder_addr))
    {
        initiator_status.target_address = responder_addr;
        return true;
    }

    return false;
}

bool initiator_is_ranging(void)
{
    return twr_is_active(&initiator_twr_ctx);
}

bool initiator_get_last_result(twr_result_t* result)
{
    if (result == NULL || !initiator_twr_ctx.last_result.valid)
    {
        return false;
    }

    *result = initiator_twr_ctx.last_result;
    return true;
}

void initiator_get_status(initiator_status_t* status)
{
    if (status != NULL)
    {
        *status       = initiator_status;
        status->state = (initiator_state_e)twr_state_machine_get_state(&initiator_twr_ctx);
        status->successful_ranges = initiator_twr_ctx.successful_transactions;
        status->failed_ranges     = initiator_twr_ctx.failed_transactions;
        status->timeout_count     = initiator_twr_ctx.timeout_count;
        status->last_result       = initiator_twr_ctx.last_result;
    }
}

void initiator_cancel_ranging(void)
{
    twr_cancel(&initiator_twr_ctx);
}
