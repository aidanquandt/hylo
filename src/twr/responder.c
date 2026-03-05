/*---------------------------------------------------------------------------
 * @file    responder.c
 * @brief   TWR Responder implementation
 *---------------------------------------------------------------------------*/
#include "responder.h"
#include "twr.h"
#include "twr_algorithm.h"
#include "twr_state_machine.h"
#include "FreeRTOS.h"
#include "error_handler.h"
#include "stopwatch.h"
#include "task.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"
#include "common.h"

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool responder_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx);
STATIC void responder_handle_message_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void responder_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void responder_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void responder_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx);
STATIC void responder_process_result_impl(twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
twr_context_t responder_twr_ctx; // Made non-static for external access
STATIC responder_status_t responder_status = {0};

STATIC const twr_callbacks_t responder_callbacks = {
    .send_message       = responder_send_message_impl,
    .handle_message     = responder_handle_message_impl,
    .handle_tx_complete = responder_handle_tx_complete_impl,
    .handle_timeout     = responder_handle_timeout_impl,
    .handle_fault       = responder_handle_fault_impl,
    .process_result     = responder_process_result_impl};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void responder_init(void)
{
    twr_state_machine_init(&responder_twr_ctx, TWR_ROLE_RESPONDER, &responder_callbacks,
                           &responder_status);
    memset(&responder_status, 0, sizeof(responder_status));
    // Address is always fetched from node module (single source of truth)
}

bool responder_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    // Start listening for incoming POLLs
    return twr_state_machine_start(&responder_twr_ctx,
                                   0); // Responder doesn't specify peer initially
}

void responder_stop(void)
{
    twr_state_machine_stop(&responder_twr_ctx);
}

void responder_get_status(responder_status_t* status)
{
    if (status != NULL)
    {
        *status                = responder_status;
        status->state          = (responder_state_e)twr_state_machine_get_state(&responder_twr_ctx);
        status->my_address     = uwb_get_address();
        status->polls_received = responder_twr_ctx.successful_transactions;
        status->responses_sent = responder_twr_ctx.successful_transactions;
        status->response_failures      = responder_twr_ctx.failed_transactions;
        status->last_initiator_address = responder_twr_ctx.peer_address;
    }
}

uint32_t responder_get_response_count(void)
{
    return responder_twr_ctx.successful_transactions;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool responder_send_message_impl(twr_msg_type_e msg_type, twr_context_t* ctx)
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

            // Include anchor position (from node module)
            vec3_t position = {0};
            if (uwb_node_get_position(&position))
            {
                response.anchor_position = position;
            }
            else
            {
                // Position unknown - send zeros
                response.anchor_position.x = 0.0f;
                response.anchor_position.y = 0.0f;
                response.anchor_position.z = 0.0f;
            }

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

            // Include anchor position (from node module)
            vec3_t position = {0};
            if (uwb_node_get_position(&position))
            {
                ack.anchor_position = position;
            }
            else
            {
                // Position unknown - send zeros
                ack.anchor_position.x = 0.0f;
                ack.anchor_position.y = 0.0f;
                ack.anchor_position.z = 0.0f;
            }

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
        error_handler_log(ERROR_SEVERITY_ERROR, "responder", "Failed to send message type %d",
                          msg_type);
        return false;
    }
}

STATIC void responder_handle_message_impl(const twr_event_t* event, twr_context_t* ctx)
{
    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;
    twr_msg_type_e msg_type         = (twr_msg_type_e)header->msg_type;

    switch (msg_type)
    {
        case TWR_MSG_POLL:
        {
            if (event->rx.length < sizeof(protocol_twr_poll_msg_t))
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "responder", "Poll too short");
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
                twr_state_machine_transition_to(ctx, TWR_STATE_SENDING);
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
                error_handler_log(ERROR_SEVERITY_WARNING, "responder", "Final too short");
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
                twr_state_machine_transition_to(ctx, TWR_STATE_SENDING);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "responder", "Failed to send final ACK");
                twr_state_machine_transition_to(ctx, TWR_STATE_WAITING); // Back to listening
            }
            break;
        }

        default:
            error_handler_log(ERROR_SEVERITY_WARNING, "responder", "Unexpected message type %d",
                              msg_type);
            break;
    }
}

STATIC void responder_handle_tx_complete_impl(const twr_event_t* event, twr_context_t* ctx)
{
    twr_msg_type_e msg_type = ctx->pending_tx_msg; // Use tracked message type

    // Store TX timestamp
    if (msg_type == TWR_MSG_RESPONSE)
    {
        ctx->timestamps[1].timestamp_dtu = event->tx.tx_timestamp;
        ctx->timestamps[1].local_time_ms = xTaskGetTickCount();

        // Wait for final
        ctx->expected_msg = TWR_MSG_FINAL;
        twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
    }
    else if (msg_type == TWR_MSG_FINAL_ACK)
    {
        // Transaction complete, back to listening
        ctx->successful_transactions++;
        ctx->expected_msg = TWR_MSG_POLL;
        ctx->peer_address = 0; // Ready to accept POLL from any initiator
        twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
    }

    ctx->pending_tx_id = 0; // Clear pending ID
}

STATIC void responder_handle_timeout_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_WARNING, "responder",
                      "Timeout waiting for message %d from initiator 0x%04X",
                      event->timeout.expected_msg, ctx->peer_address);

    // For responder, timeout means transaction failed - return to listening
    ctx->failed_transactions++;
    ctx->expected_msg = TWR_MSG_POLL;
    ctx->peer_address = 0; // Ready to accept POLL from any initiator
    twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
}

STATIC void responder_handle_fault_impl(const twr_event_t* event, twr_context_t* ctx)
{
    error_handler_log(ERROR_SEVERITY_ERROR, "responder", "Fault: %d - %s", event->fault.fault_code,
                      event->fault.description);

    ctx->fault_code = event->fault.fault_code;
    ctx->failed_transactions++;

    // Return to listening state
    ctx->expected_msg = TWR_MSG_POLL;
    twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
}

STATIC void responder_process_result_impl(twr_context_t* ctx)
{
    // Responder doesn't process distance results, just completes transaction
    // Distance calculation is done by the initiator
    error_handler_log(ERROR_SEVERITY_INFO, "responder",
                      "Completed ranging transaction with initiator 0x%04X", ctx->peer_address);
}
