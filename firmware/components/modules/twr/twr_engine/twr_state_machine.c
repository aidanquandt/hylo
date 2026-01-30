/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_state_machine.h"
#include "error_handler.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
void twr_state_machine_transition_to(twr_context_t* ctx, twr_state_e new_state);
STATIC void twr_handle_timeout(twr_context_t* ctx);
STATIC bool twr_validate_message(const twr_event_t* event, twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

void twr_state_machine_transition_to(twr_context_t* ctx, twr_state_e new_state)
{
    if (ctx->state != new_state)
    {
        ctx->state       = new_state;
        ctx->state_timer = 0;

        if (new_state == TWR_STATE_PROCESSING)
        {
            ctx->callbacks->process_result(ctx);
            twr_state_machine_transition_to(ctx, TWR_STATE_IDLE);
        }
    }
}

STATIC void twr_handle_timeout(twr_context_t* ctx)
{
    twr_event_t timeout_event = {
        .type    = TWR_EVENT_TIMEOUT,
        .timeout = {.elapsed_ms = ctx->state_timer, .expected_msg = ctx->expected_msg}};

    ctx->timeout_count++;
    ctx->callbacks->handle_timeout(&timeout_event, ctx);
}

STATIC bool twr_validate_message(const twr_event_t* event, twr_context_t* ctx)
{
    if (event->rx.length < sizeof(protocol_header_t))
    {
        return false;
    }

    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;

    if (header->protocol_type != PROTOCOL_TYPE_TWR)
    {
        return false;
    }

    twr_msg_type_e rx_msg_type = (twr_msg_type_e)header->msg_type;
    if (rx_msg_type != ctx->expected_msg)
    {
        return false;
    }

    if (!(ctx->role == TWR_ROLE_RESPONDER && rx_msg_type == TWR_MSG_POLL))
    {
        if (header->sequence != ctx->sequence)
        {
            return false;
        }
    }

    if (ctx->peer_address != 0 && event->rx.src_addr != ctx->peer_address)
    {
        return false;
    }

    if (ctx->role == TWR_ROLE_RESPONDER && rx_msg_type == TWR_MSG_POLL)
    {
        ctx->peer_address = event->rx.src_addr;
        ctx->sequence     = header->sequence;
    }

    return true;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void twr_state_machine_init(twr_context_t* ctx, twr_role_e role, const twr_callbacks_t* callbacks,
                            void* role_context)
{
    if (ctx == NULL || callbacks == NULL)
    {
        return;
    }

    memset(ctx, 0, sizeof(twr_context_t));
    ctx->role           = role;
    ctx->callbacks      = callbacks;
    ctx->role_context   = role_context;
    ctx->state          = TWR_STATE_IDLE;
    ctx->pending_tx_msg = TWR_MSG_POLL; // Initialize to a default value
}

bool twr_state_machine_start(twr_context_t* ctx, uint16_t peer_addr)
{
    if (ctx == NULL || ctx->state != TWR_STATE_IDLE)
    {
        return false;
    }

    ctx->peer_address = peer_addr;
    ctx->sequence++;
    ctx->retry_count    = 0;
    ctx->fault_code     = TWR_FAULT_NONE;
    ctx->state_timer    = 0;
    ctx->pending_tx_id  = 0;            // Clear any stale TX ID
    ctx->pending_tx_msg = TWR_MSG_POLL; // Reset message type

    // Clear timestamps
    memset(ctx->timestamps, 0, sizeof(ctx->timestamps));
    memset(ctx->remote_timestamps, 0, sizeof(ctx->remote_timestamps));

    if (ctx->role == TWR_ROLE_INITIATOR)
    {
        ctx->expected_msg = TWR_MSG_RESPONSE;

        if (ctx->callbacks->send_message(TWR_MSG_POLL, ctx))
        {
            twr_state_machine_transition_to(ctx, TWR_STATE_SENDING);
            return true;
        }
        else
        {
            ctx->fault_code = TWR_FAULT_SEND_FAILED;
            return false;
        }
    }
    else
    {
        ctx->expected_msg = TWR_MSG_POLL;
        twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
        return true;
    }
}

void twr_state_machine_stop(twr_context_t* ctx)
{
    if (ctx != NULL)
    {
        twr_state_machine_transition_to(ctx, TWR_STATE_IDLE);
    }
}

void twr_state_machine_process(twr_context_t* ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    ctx->state_timer++;

    if (ctx->state == TWR_STATE_WAITING && ctx->peer_address != 0)
    {
        if (ctx->state_timer >= TWR_TIMEOUT_MS)
        {
            twr_handle_timeout(ctx);
        }
    }
    else if (ctx->state == TWR_STATE_SENDING)
    {
        if (ctx->state_timer >= TWR_TX_COMPLETION_TIMEOUT_MS) // Safety timeout for TX completion
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                              "TX completion timeout in SENDING state, resetting");

            ctx->timeout_count++;
            ctx->failed_transactions++;

            if (ctx->role == TWR_ROLE_RESPONDER)
            {
                ctx->expected_msg = TWR_MSG_POLL;
                ctx->peer_address = 0; // Ready to accept POLL from any initiator
                twr_state_machine_transition_to(ctx, TWR_STATE_WAITING);
            }
            else
            {
                twr_state_machine_transition_to(ctx, TWR_STATE_IDLE);
            }
        }
    }
}

void twr_state_machine_handle_event(twr_context_t* ctx, const twr_event_t* event)
{
    if (ctx == NULL || event == NULL)
    {
        return;
    }

    switch (event->type)
    {
        case TWR_EVENT_TX_COMPLETE:
            if (ctx->state == TWR_STATE_SENDING && event->tx.message_id == ctx->pending_tx_id)
            {
                ctx->callbacks->handle_tx_complete(event, ctx);
            }
            else if (ctx->state != TWR_STATE_SENDING)
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                                  "TX_COMPLETE: wrong state (%d), expected SENDING(%d)", ctx->state,
                                  TWR_STATE_SENDING);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                                  "TX_COMPLETE: msg_id mismatch (exp:%lu, got:%lu)",
                                  ctx->pending_tx_id, event->tx.message_id);
            }
            break;

        case TWR_EVENT_RX_MESSAGE:
            if (ctx->state == TWR_STATE_WAITING && twr_validate_message(event, ctx))
            {
                ctx->callbacks->handle_message(event, ctx);
            }
            else
            {
                // Extract message type for logging
                twr_msg_type_e rx_msg_type = TWR_MSG_POLL; // Default
                if (event->rx.length >= sizeof(protocol_header_t))
                {
                    const protocol_header_t* header = (const protocol_header_t*)event->rx.data;
                    rx_msg_type                     = (twr_msg_type_e)header->msg_type;
                }

                error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                                  "RX_MESSAGE: wrong state (%d) or invalid msg_type=%d from 0x%04X",
                                  ctx->state, rx_msg_type, event->rx.src_addr);
            }
            break;

        case TWR_EVENT_TIMEOUT:
            if (ctx->state == TWR_STATE_WAITING)
            {
                ctx->callbacks->handle_timeout(event, ctx);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                                  "TIMEOUT: unexpected in state %d", ctx->state);
            }
            break;

        case TWR_EVENT_FAULT:
            ctx->callbacks->handle_fault(event, ctx);
            break;
    }
}

twr_state_e twr_state_machine_get_state(const twr_context_t* ctx)
{
    return (ctx != NULL) ? ctx->state : TWR_STATE_IDLE;
}