/*---------------------------------------------------------------------------
 * @file    tag.c
 * @brief   Unified TWR Tag implementation using generic state machine
 * @details Tag implementation using the unified TWR state machine
 *---------------------------------------------------------------------------*/

#include "tag.h"
#include "../twr.h" // For registration functions
#include "FreeRTOS.h"
#include "error_handler.h"
#include "stopwatch.h"
#include "task.h"
#include "twr/twr_mode.h" // Simple mode management
#include "twr_algorithm.h"
#include "twr_state_machine.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/
#define TWR_MAX_RETRIES 2U

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
twr_context_t tag_twr_ctx; // Made non-static for external access
STATIC tag_status_t tag_status = {0};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool tag_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx);
STATIC void tag_handle_message_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void tag_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void tag_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void tag_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void tag_process_result_impl(twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Role-Specific Callbacks
 *---------------------------------------------------------------------------*/
STATIC const twr_callbacks_t tag_callbacks = {.send_message       = tag_send_message_impl,
                                              .handle_message     = tag_handle_message_impl,
                                              .handle_tx_complete = tag_handle_tx_complete_impl,
                                              .handle_timeout     = tag_handle_timeout_impl,
                                              .handle_fault       = tag_handle_fault_impl,
                                              .process_result     = tag_process_result_impl};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void tag_init(void)
{
    twr_init(&tag_twr_ctx, TWR_ROLE_TAG, &tag_callbacks, &tag_status);
    memset(&tag_status, 0, sizeof(tag_status));
}

bool tag_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    // Only start if we're in tag mode
    if (twr_mode_get_current() != TWR_MODE_TAG)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Not in tag mode");
        return false;
    }

    // No registration needed - ranging.c dispatches based on mode
    return true;
}

void tag_stop(void)
{
    twr_stop(&tag_twr_ctx);
}

bool tag_start_ranging(uint16_t anchor_addr)
{
    if (!uwb_is_ready())
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "UWB not ready");
        return false;
    }

    stopwatch_start(0);

    if (twr_start(&tag_twr_ctx, anchor_addr))
    {
        tag_status.target_address = anchor_addr;
        return true;
    }

    return false;
}

bool tag_is_ranging(void)
{
    return twr_is_active(&tag_twr_ctx);
}

bool tag_get_last_result(twr_result_t* result)
{
    if (result == NULL || !tag_twr_ctx.last_result.valid)
    {
        return false;
    }

    *result = tag_twr_ctx.last_result;
    return true;
}

void tag_get_status(tag_status_t* status)
{
    if (status != NULL)
    {
        *status                   = tag_status;
        status->state             = (tag_state_e)twr_get_state(&tag_twr_ctx);
        status->successful_ranges = tag_twr_ctx.successful_transactions;
        status->failed_ranges     = tag_twr_ctx.failed_transactions;
        status->timeout_count     = tag_twr_ctx.timeout_count;
        status->last_result       = tag_twr_ctx.last_result;
    }
}

void tag_cancel_ranging(void)
{
    twr_cancel(&tag_twr_ctx);
}

/*---------------------------------------------------------------------------
 * Private Callback Implementations
 *---------------------------------------------------------------------------*/

STATIC bool tag_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx)
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
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Failed to send message type %d", msg_type);
        return false;
    }
}

STATIC void tag_handle_message_impl(const twr_event_t* event, twr_context_t* ctx)
{
    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;
    twr_msg_type_e msg_type         = (twr_msg_type_e)header->msg_type;

    switch (msg_type)
    {
        case TWR_MSG_RESPONSE:
        {
            if (event->rx.length < sizeof(protocol_twr_response_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Response too short");
                return;
            }

            const protocol_twr_response_msg_t* resp =
                (const protocol_twr_response_msg_t*)event->rx.data;

            // Store response RX timestamp
            ctx->timestamps[1].timestamp_dtu = event->rx.rx_timestamp;
            ctx->timestamps[1].local_time_ms = xTaskGetTickCount();

            // Store anchor's poll RX timestamp
            ctx->remote_timestamps[0] = twr_timestamp_to_u64(resp->poll_rx_ts);

            // Prepare for final
            ctx->expected_msg = TWR_MSG_FINAL_ACK;

            // Send final
            if (ctx->callbacks->send_message(TWR_MSG_FINAL, ctx))
            {
                twr_transition_to(ctx, TWR_STATE_SENDING);
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
                error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Final ACK too short");
                return;
            }

            const protocol_twr_final_ack_msg_t* ack =
                (const protocol_twr_final_ack_msg_t*)event->rx.data;

            // Store anchor's timestamps
            ctx->remote_timestamps[1] = twr_timestamp_to_u64(ack->resp_tx_ts);  // Response TX
            ctx->remote_timestamps[2] = twr_timestamp_to_u64(ack->final_rx_ts); // Final RX

            // Process the result
            twr_transition_to(ctx, TWR_STATE_PROCESSING);
            break;
        }

        default:
            error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Unexpected message type %d",
                              msg_type);
            break;
    }
}

STATIC void tag_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx)
{
    twr_msg_type_e msg_type = ctx->pending_tx_msg; // Use tracked message type

    // Store TX timestamp
    if (msg_type == TWR_MSG_POLL)
    {
        ctx->timestamps[0].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[0].local_time_ms = xTaskGetTickCount();

        // Wait for response
        ctx->expected_msg = TWR_MSG_RESPONSE;
        twr_transition_to(ctx, TWR_STATE_WAITING);
    }
    else if (msg_type == TWR_MSG_FINAL)
    {
        ctx->timestamps[2].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[2].local_time_ms = xTaskGetTickCount();

        // Wait for final ACK
        ctx->expected_msg = TWR_MSG_FINAL_ACK;
        twr_transition_to(ctx, TWR_STATE_WAITING);
    }

    ctx->pending_tx_id = 0; // Clear pending ID
}

STATIC void tag_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Timeout waiting for message %d",
                      event->timeout.expected_msg);

    // Retry logic
    if (ctx->retry_count < TWR_MAX_RETRIES)
    {
        ctx->retry_count++;
        error_handler_log(ERROR_SEVERITY_INFO, "tag", "Retrying (attempt %u)", ctx->retry_count);

        // Restart with POLL
        ctx->expected_msg = TWR_MSG_RESPONSE;

        if (ctx->callbacks->send_message(TWR_MSG_POLL, ctx))
        {
            twr_transition_to(ctx, TWR_STATE_SENDING);
        }
        else
        {
            twr_transition_to(ctx, TWR_STATE_IDLE);
        }
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Max retries exceeded");
        ctx->failed_transactions++;
        twr_transition_to(ctx, TWR_STATE_IDLE);
    }
}

STATIC void tag_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Fault: %d - %s", event->fault.fault_code,
                      event->fault.description);

    ctx->fault_code = event->fault.fault_code;
    ctx->failed_transactions++;

    twr_transition_to(ctx, TWR_STATE_IDLE);
}

STATIC void tag_process_result_impl(twr_context_t* ctx)
{
    twr_result_t result = {0};

    // Calculate distance using DS-TWR with 6 timestamps
    twr_status_e status =
        twr_calculate_ds_twr(ctx->timestamps[0].timestamp_dtu, // Poll TX (tag)
                             ctx->remote_timestamps[0],        // Poll RX (anchor)
                             ctx->remote_timestamps[1],        // Response TX (anchor)
                             ctx->timestamps[1].timestamp_dtu, // Response RX (tag)
                             ctx->timestamps[2].timestamp_dtu, // Final TX (tag)
                             ctx->remote_timestamps[2],        // Final RX (anchor)
                             &result);

    if (status == TWR_SUCCESS && result.valid)
    {
        stopwatch_stop(0);
        uint32_t elapsed_us = stopwatch_elapsed_us(0);

        result.remote_addr  = ctx->peer_address;
        result.timestamp_ms = xTaskGetTickCount();

        ctx->last_result = result;
        ctx->successful_transactions++;

        error_handler_log(ERROR_SEVERITY_INFO, "tag", "Ranging success: %.2f m, %lu us",
                          result.distance_m, (unsigned long)elapsed_us);
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Distance calculation failed: %d", status);
        ctx->failed_transactions++;
    }
}