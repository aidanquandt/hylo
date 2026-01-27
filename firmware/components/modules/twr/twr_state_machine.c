/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_state_machine.h"
#include "error_handler.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/
void twr_transition_to(twr_context_t* ctx, twr_state_e new_state);

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void twr_handle_timeout(twr_context_t* ctx);
STATIC bool twr_validate_message(const twr_event_t* event, twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

void twr_transition_to(twr_context_t* ctx, twr_state_e new_state)
{
    if (ctx->state != new_state)
    {
        ctx->state       = new_state;
        ctx->state_timer = 0;

        // Handle state entry actions
        if (new_state == TWR_STATE_PROCESSING)
        {
            ctx->callbacks->process_result(ctx);
            // Auto-transition back to idle after processing
            twr_transition_to(ctx, TWR_STATE_IDLE);
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

    // Check protocol type
    if (header->protocol_type != PROTOCOL_TYPE_TWR)
    {
        return false;
    }

    // Check message type matches expected
    twr_msg_type_e rx_msg_type = (twr_msg_type_e)header->msg_type;
    if (rx_msg_type != ctx->expected_msg)
    {
        return false;
    }

    // Check sequence (except for POLL to anchor)
    if (!(ctx->role == TWR_ROLE_ANCHOR && rx_msg_type == TWR_MSG_POLL))
    {
        if (header->sequence != ctx->sequence)
        {
            return false;
        }
    }

    // Check source address - anchor accepts any source for POLL
    if (ctx->peer_address != 0 && event->rx.src_addr != ctx->peer_address)
    {
        return false;
    }

    // For anchor receiving POLL, capture sender info
    if (ctx->role == TWR_ROLE_ANCHOR && rx_msg_type == TWR_MSG_POLL)
    {
        ctx->peer_address = event->rx.src_addr;
        ctx->sequence     = header->sequence;
    }

    return true;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void twr_init(twr_context_t* ctx, twr_role_e role, const twr_callbacks_t* callbacks,
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

bool twr_start(twr_context_t* ctx, uint16_t peer_addr)
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

    if (ctx->role == TWR_ROLE_TAG)
    {
        // Tag initiates with POLL
        ctx->expected_msg = TWR_MSG_RESPONSE;

        if (ctx->callbacks->send_message(TWR_MSG_POLL, ctx))
        {
            twr_transition_to(ctx, TWR_STATE_SENDING);
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
        // Anchor waits for POLL
        ctx->expected_msg = TWR_MSG_POLL;
        twr_transition_to(ctx, TWR_STATE_WAITING);
        return true;
    }
}

void twr_stop(twr_context_t* ctx)
{
    if (ctx != NULL)
    {
        twr_transition_to(ctx, TWR_STATE_IDLE);
    }
}

void twr_process(twr_context_t* ctx)
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
    // Add timeout for SENDING state to prevent getting stuck
    else if (ctx->state == TWR_STATE_SENDING)
    {
        if (ctx->state_timer >= 10) // 10ms timeout for TX completion
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                              "TX completion timeout in SENDING state, resetting");

            ctx->timeout_count++;
            ctx->failed_transactions++;

            // Reset to waiting state for anchors, idle for tags
            if (ctx->role == TWR_ROLE_ANCHOR)
            {
                ctx->expected_msg = TWR_MSG_POLL;
                ctx->peer_address = 0; // Ready to accept POLL from any tag
                twr_transition_to(ctx, TWR_STATE_WAITING);
            }
            else
            {
                twr_transition_to(ctx, TWR_STATE_IDLE);
            }
        }
    }
}

void twr_handle_event(twr_context_t* ctx, const twr_event_t* event)
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
                error_handler_log(ERROR_SEVERITY_WARNING, "twr_sm",
                                  "RX_MESSAGE: wrong state (%d) or invalid message from 0x%04X",
                                  ctx->state, event->rx.src_addr);
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

twr_state_e twr_get_state(const twr_context_t* ctx)
{
    return (ctx != NULL) ? ctx->state : TWR_STATE_IDLE;
}

bool twr_is_active(const twr_context_t* ctx)
{
    return (ctx != NULL) && (ctx->state != TWR_STATE_IDLE);
}

void twr_cancel(twr_context_t* ctx)
{
    if (ctx != NULL)
    {
        twr_transition_to(ctx, TWR_STATE_IDLE);
    }
}

void twr_rx_callback(twr_context_t* ctx, const uint8_t* data, uint16_t length, uint16_t src_addr,
                     uint64_t rx_timestamp)
{
    if (ctx == NULL)
        return;

    twr_event_t event = {
        .type = TWR_EVENT_RX_MESSAGE,
        .rx = {.data = data, .length = length, .src_addr = src_addr, .rx_timestamp = rx_timestamp}};

    twr_handle_event(ctx, &event);
}

void twr_tx_done_callback(twr_context_t* ctx, uint32_t message_id, uint64_t tx_timestamp)
{
    if (ctx == NULL)
        return;

    twr_event_t event = {
        .type = TWR_EVENT_TX_COMPLETE,
        .tx   = {
              .message_id   = message_id,
              .tx_timestamp = tx_timestamp,
              .msg_type     = TWR_MSG_POLL // Placeholder - actual type determined in callback
        }};

    twr_handle_event(ctx, &event);
}