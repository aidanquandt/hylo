/*---------------------------------------------------------------------------
 * @file    anchor.c
 * @brief   Unified TWR Anchor implementation using generic state machine
 * @details Anchor implementation using the unified TWR state machine
 *---------------------------------------------------------------------------*/

#include "anchor.h"
#include "../twr.h" // For registration functions
#include "FreeRTOS.h"
#include "error_handler.h"
#include "task.h"
#include "twr/twr_mode.h" // Simple mode management
#include "twr_state_machine.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
twr_context_t anchor_twr_ctx; // Made non-static for external access
STATIC anchor_status_t anchor_status = {0};
STATIC uint16_t anchor_address       = 0x1234; // Default address

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool anchor_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx);
STATIC void anchor_handle_message_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void anchor_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void anchor_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void anchor_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void anchor_process_result_impl(twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Role-Specific Callbacks
 *---------------------------------------------------------------------------*/
STATIC const twr_callbacks_t anchor_callbacks = {
    .send_message       = anchor_send_message_impl,
    .handle_message     = anchor_handle_message_impl,
    .handle_tx_complete = anchor_handle_tx_complete_impl,
    .handle_timeout     = anchor_handle_timeout_impl,
    .handle_fault       = anchor_handle_fault_impl,
    .process_result     = anchor_process_result_impl};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void anchor_init(void)
{
    twr_init(&anchor_twr_ctx, TWR_ROLE_ANCHOR, &anchor_callbacks, &anchor_status);
    memset(&anchor_status, 0, sizeof(anchor_status));
    anchor_status.my_address = anchor_address;
}

bool anchor_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    // Only start if we're in anchor mode
    if (twr_mode_get_current() != TWR_MODE_ANCHOR)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Not in anchor mode");
        return false;
    }

    // No registration needed - ranging.c dispatches based on mode
    return twr_start(&anchor_twr_ctx, 0); // Anchor doesn't specify peer initially
}

void anchor_stop(void)
{
    twr_stop(&anchor_twr_ctx);
}

void anchor_set_address(uint16_t address)
{
    anchor_address           = address;
    anchor_status.my_address = address;
}

uint16_t anchor_get_address(void)
{
    return anchor_address;
}

void anchor_get_status(anchor_status_t* status)
{
    if (status != NULL)
    {
        *status                   = anchor_status;
        status->state             = (anchor_state_e)twr_get_state(&anchor_twr_ctx);
        status->polls_received    = anchor_twr_ctx.successful_transactions;
        status->responses_sent    = anchor_twr_ctx.successful_transactions;
        status->response_failures = anchor_twr_ctx.failed_transactions;
        status->last_tag_address  = anchor_twr_ctx.peer_address;
    }
}

uint32_t anchor_get_response_count(void)
{
    return anchor_twr_ctx.successful_transactions;
}

/*---------------------------------------------------------------------------
 * Private Callback Implementations
 *---------------------------------------------------------------------------*/

STATIC bool anchor_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx)
{
    uwb_send_result_t result;

    switch (msg_type)
    {
        case TWR_MSG_RESPONSE:
        {
            protocol_twr_response_msg_t response = {.header = {.protocol_type = PROTOCOL_TYPE_TWR,
                                                               .msg_type      = TWR_MSG_RESPONSE,
                                                               .sequence      = ctx->sequence}};

            // Include poll RX timestamp
            twr_u64_to_timestamp(ctx->timestamps[0].timestamp_dtu, response.poll_rx_ts);

            result = uwb_send_message((uint8_t*)&response, sizeof(response), ctx->peer_address);
            break;
        }

        case TWR_MSG_FINAL_ACK:
        {
            protocol_twr_final_ack_msg_t ack = {.header = {.protocol_type = PROTOCOL_TYPE_TWR,
                                                           .msg_type      = TWR_MSG_FINAL_ACK,
                                                           .sequence      = ctx->sequence}};

            // Include response TX and final RX timestamps
            twr_u64_to_timestamp(ctx->timestamps[1].timestamp_dtu, ack.resp_tx_ts);
            twr_u64_to_timestamp(ctx->timestamps[2].timestamp_dtu, ack.final_rx_ts);

            result = uwb_send_message((uint8_t*)&ack, sizeof(ack), ctx->peer_address);
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
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "Failed to send message type %d",
                          msg_type);
        return false;
    }
}

STATIC void anchor_handle_message_impl(const twr_event_t* event, twr_context_t* ctx)
{
    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;
    twr_msg_type_e msg_type         = (twr_msg_type_e)header->msg_type;

    switch (msg_type)
    {
        case TWR_MSG_POLL:
        {
            if (event->rx.length < sizeof(protocol_twr_poll_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Poll too short");
                return;
            }

            // Store poll RX timestamp
            ctx->timestamps[0].timestamp_dtu = event->rx.rx_timestamp;
            ctx->timestamps[0].local_time_ms = xTaskGetTickCount();

            // Prepare response
            ctx->expected_msg = TWR_MSG_FINAL;

            // Send response
            if (ctx->callbacks->send_message(TWR_MSG_RESPONSE, ctx))
            {
                twr_transition_to(ctx, TWR_STATE_SENDING);
            }
            else
            {
                ctx->fault_code         = TWR_FAULT_SEND_FAILED;
                twr_event_t fault_event = {.type  = TWR_EVENT_FAULT,
                                           .fault = {.fault_code  = TWR_FAULT_SEND_FAILED,
                                                     .description = "Response send failed"}};
                ctx->callbacks->handle_fault(&fault_event, ctx);
            }
            break;
        }

        case TWR_MSG_FINAL:
        {
            if (event->rx.length < sizeof(protocol_twr_final_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Final too short");
                return;
            }

            // Store final RX timestamp
            ctx->timestamps[2].timestamp_dtu = event->rx.rx_timestamp;
            ctx->timestamps[2].local_time_ms = xTaskGetTickCount();

            // Prepare final ACK
            ctx->expected_msg = TWR_MSG_POLL; // Ready for next transaction

            // Send final ACK
            if (ctx->callbacks->send_message(TWR_MSG_FINAL_ACK, ctx))
            {
                twr_transition_to(ctx, TWR_STATE_SENDING);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "Failed to send final ACK");
                twr_transition_to(ctx, TWR_STATE_WAITING); // Back to listening
            }
            break;
        }

        default:
            error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Unexpected message type %d",
                              msg_type);
            break;
    }
}

STATIC void anchor_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx)
{
    twr_msg_type_e msg_type = ctx->pending_tx_msg; // Use tracked message type

    // Store TX timestamp
    if (msg_type == TWR_MSG_RESPONSE)
    {
        ctx->timestamps[1].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[1].local_time_ms = xTaskGetTickCount();

        // Wait for final
        ctx->expected_msg = TWR_MSG_FINAL;
        twr_transition_to(ctx, TWR_STATE_WAITING);
    }
    else if (msg_type == TWR_MSG_FINAL_ACK)
    {
        // Transaction complete, back to listening
        ctx->successful_transactions++;
        ctx->expected_msg = TWR_MSG_POLL;
        ctx->peer_address = 0; // Ready to accept POLL from any tag
        twr_transition_to(ctx, TWR_STATE_WAITING);
    }

    ctx->pending_tx_id = 0; // Clear pending ID
}

STATIC void anchor_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                      "Timeout waiting for message %d from tag 0x%04X", event->timeout.expected_msg,
                      ctx->peer_address);

    // For anchor, timeout means transaction failed - return to listening
    ctx->failed_transactions++;
    ctx->expected_msg = TWR_MSG_POLL;
    ctx->peer_address = 0; // Ready to accept POLL from any tag
    twr_transition_to(ctx, TWR_STATE_WAITING);
}

STATIC void anchor_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "Fault: %d - %s", event->fault.fault_code,
                      event->fault.description);

    ctx->fault_code = event->fault.fault_code;
    ctx->failed_transactions++;

    // Return to listening state
    ctx->expected_msg = TWR_MSG_POLL;
    twr_transition_to(ctx, TWR_STATE_WAITING);
}

STATIC void anchor_process_result_impl(twr_context_t* ctx)
{
    // Anchor doesn't process distance results, just completes transaction
    // Distance calculation is done by the tag
    error_handler_log(ERROR_SEVERITY_INFO, "anchor",
                      "Completed ranging transaction with tag 0x%04X", ctx->peer_address);
}