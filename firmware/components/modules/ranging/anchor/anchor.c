/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "anchor.h"
#include "error_handler.h"
#include "platform_os.h"
#include "state_machine.h"
#include "twr/twr_types.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/
#define ANCHOR_FINAL_TIMEOUT_TICKS (TWR_RESPONSE_TIMEOUT_MS) // Timeout waiting for FINAL

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/

typedef enum
{
    ANCHOR_FAULT_NONE = 0,
    ANCHOR_FAULT_UWB_NOT_READY,
    ANCHOR_FAULT_SEND_FAILED,
    ANCHOR_FAULT_INVALID_POLL
} anchor_fault_e;

typedef struct
{
    bool active; // Anchor is active

    // Current ranging context
    bool processing_poll;   // Currently processing a poll
    uint16_t tag_address;   // Tag we're responding to
    uint16_t sequence;      // Current sequence number
    uint32_t pending_tx_id; // Message ID of pending transmission

    // Timestamps
    twr_timestamp_t poll_rx; // When we received poll
    uint64_t resp_tx;        // Actual TX timestamp of response (measured after transmission)

    // Statistics
    uint32_t polls_received;
    uint32_t responses_sent;
    uint32_t response_failures;
    uint16_t last_tag_address;

    anchor_fault_e fault_code;
} anchor_context_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC uint16_t anchor_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void anchor_state_wait_poll_on_entry(uint16_t prevState);
STATIC void anchor_state_wait_final_on_entry(uint16_t prevState);
STATIC void anchor_state_faulted_on_entry(uint16_t prevState);
STATIC void anchor_handle_poll(const uint8_t* data, uint16_t length, uint16_t src_addr,
                               uint64_t rx_timestamp);
STATIC void anchor_handle_final(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                uint64_t rx_timestamp);
STATIC bool anchor_send_response(void);
STATIC bool anchor_send_final_ack(uint16_t tag_addr, uint16_t sequence, uint64_t resp_tx_ts,
                                  uint64_t final_rx_ts);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC anchor_context_t anchor_ctx = {0};

STATIC const state_s anchor_states[] = {
    [ANCHOR_STATE_IDLE]              = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_WAIT_POLL]         = {.process = NULL,
                                        .onEntry = anchor_state_wait_poll_on_entry,
                                        .onExit  = NULL},
    [ANCHOR_STATE_SENDING_RESPONSE]  = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_WAIT_FINAL]        = {.process = NULL,
                                        .onEntry = anchor_state_wait_final_on_entry,
                                        .onExit  = NULL},
    [ANCHOR_STATE_SENDING_FINAL_ACK] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_FAULTED]           = {
                  .process = NULL, .onEntry = anchor_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s anchor_sm = {.prev_state      = ANCHOR_STATE_IDLE,
                                    .curr_state      = ANCHOR_STATE_IDLE,
                                    .next_state      = ANCHOR_STATE_IDLE,
                                    .timer           = 0,
                                    .transitionLogic = anchor_transition_logic,
                                    .states          = anchor_states};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void anchor_state_wait_poll_on_entry(uint16_t prevState)
{
    (void)prevState;

    // Clear any previous transaction context
    anchor_ctx.processing_poll = false;
    anchor_ctx.tag_address     = 0;
    anchor_ctx.sequence        = 0;
    anchor_ctx.fault_code      = ANCHOR_FAULT_NONE;
}

STATIC void anchor_state_wait_final_on_entry(uint16_t prevState)
{
    (void)prevState;

    // Timer automatically reset by state machine framework
}

STATIC uint16_t anchor_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    switch (currentState)
    {
        case ANCHOR_STATE_WAIT_FINAL:
            // Check for timeout waiting for FINAL message
            if (stateTimer >= ANCHOR_FINAL_TIMEOUT_TICKS)
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                                  "Timeout waiting for FINAL from tag 0x%04X",
                                  anchor_ctx.tag_address);
                return ANCHOR_STATE_WAIT_POLL; // Ready for next tag
            }
            break;

        case ANCHOR_STATE_FAULTED:
            // Auto-recover from faults by returning to WAIT_POLL
            return ANCHOR_STATE_WAIT_POLL;

        default:
            // No automatic transitions for other states
            break;
    }

    // Stay in current state by default
    return currentState;
}

STATIC void anchor_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Fault: %d", anchor_ctx.fault_code);

    // Clear fault for next attempt
    anchor_ctx.fault_code = ANCHOR_FAULT_NONE;

    // Will auto-recover to WAIT_POLL in process_1kHz
}

STATIC void anchor_handle_poll(const uint8_t* data, uint16_t length, uint16_t src_addr,
                               uint64_t rx_timestamp)
{
    // Validate message
    if (length < sizeof(protocol_twr_poll_msg_t))
    {
        anchor_ctx.fault_code = ANCHOR_FAULT_INVALID_POLL;
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "POLL validation FAILED - too short");
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_FAULTED);
        return;
    }

    const protocol_twr_poll_msg_t* poll = (const protocol_twr_poll_msg_t*)data;

    // Record poll details and clear old timestamps
    anchor_ctx.tag_address     = src_addr;
    anchor_ctx.sequence        = poll->header.sequence;
    anchor_ctx.processing_poll = true;
    anchor_ctx.pending_tx_id   = 0; // Will be set when sending response

    // Clear previous ranging timestamps
    memset(&anchor_ctx.poll_rx, 0, sizeof(anchor_ctx.poll_rx));
    anchor_ctx.resp_tx = 0;

    // Use the rx_timestamp passed with this message (no race condition)
    anchor_ctx.poll_rx.timestamp_dtu = rx_timestamp;

    anchor_ctx.polls_received++;

    // Send response and transition to SENDING_RESPONSE
    if (!anchor_send_response())
    {
        anchor_ctx.fault_code = ANCHOR_FAULT_SEND_FAILED;
        anchor_ctx.response_failures++;
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor",
                          "POLL handling FAILED - response send error");
        anchor_ctx.processing_poll = false;
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_FAULTED);
    }
    else
    {
        // Transition to SENDING_RESPONSE - will return to LISTENING when TX done
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_SENDING_RESPONSE);
    }
}

STATIC bool anchor_send_response(void)
{
    protocol_twr_response_msg_t response;
    response.header.protocol_type = PROTOCOL_TYPE_TWR;
    response.header.msg_type      = TWR_MSG_TYPE_RESPONSE;
    response.header.sequence      = anchor_ctx.sequence;

    // Store poll RX timestamp in response (in 5-byte format for transmission)
    twr_u64_to_timestamp(anchor_ctx.poll_rx.timestamp_dtu, response.poll_rx_ts);

    // Send response immediately - actual TX timestamp captured in tx_done_callback
    uwb_send_result_t result =
        uwb_send_message((uint8_t*)&response, sizeof(response), anchor_ctx.tag_address);
    if (!result.success)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "RESPONSE TX FAILED");
        return false;
    }

    // Store message ID to validate TX callback
    anchor_ctx.pending_tx_id = result.message_id;

    return true;
}

STATIC void anchor_handle_final(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                uint64_t rx_timestamp)
{
    // Validate message
    if (length < sizeof(protocol_twr_final_msg_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "FINAL REJECT - too short (%u < %u)",
                          length, sizeof(protocol_twr_final_msg_t));
        anchor_ctx.fault_code = ANCHOR_FAULT_INVALID_POLL;
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_FAULTED);
        return;
    }

    const protocol_twr_final_msg_t* final = (const protocol_twr_final_msg_t*)data;

    // Verify sequence matches
    if (final->header.sequence != anchor_ctx.sequence)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                          "FINAL REJECT - sequence mismatch (got %u, expected %u)",
                          final->header.sequence, anchor_ctx.sequence);
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
        return;
    }

    // Use the rx_timestamp passed with this message (no race condition)
    uint64_t final_rx_ts_dtu = rx_timestamp;

    // Send FINAL_ACK and transition to SENDING_FINAL_ACK
    if (!anchor_send_final_ack(src_addr, final->header.sequence, anchor_ctx.resp_tx,
                               final_rx_ts_dtu))
    {
        anchor_ctx.fault_code = ANCHOR_FAULT_SEND_FAILED;
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_FAULTED);
    }
    else
    {
        // Transition to SENDING_FINAL_ACK - will return to WAIT_POLL when TX done
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_SENDING_FINAL_ACK);
    }
}

STATIC bool anchor_send_final_ack(uint16_t tag_addr, uint16_t sequence, uint64_t resp_tx_ts,
                                  uint64_t final_rx_ts)
{
    protocol_twr_final_ack_msg_t ack;
    ack.header.protocol_type = PROTOCOL_TYPE_TWR;
    ack.header.msg_type      = TWR_MSG_TYPE_FINAL_ACK;
    ack.header.sequence      = sequence;

    // Encode anchor's response TX timestamp (measured from earlier) in the ACK message
    twr_u64_to_timestamp(resp_tx_ts, ack.resp_tx_ts);

    // Encode anchor's final RX timestamp in the ACK message
    twr_u64_to_timestamp(final_rx_ts, ack.final_rx_ts);

    // Send ACK immediately (no delayed TX needed for ACK)
    uwb_send_result_t result = uwb_send_message((uint8_t*)&ack, sizeof(ack), tag_addr);
    if (!result.success)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "Failed to send ACK to 0x%04X", tag_addr);
        return false;
    }

    // Store message ID for TX callback validation (FINAL_ACK doesn't need timestamp)
    anchor_ctx.pending_tx_id = result.message_id;

    return true;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void anchor_init(void)
{
    memset(&anchor_ctx, 0, sizeof(anchor_ctx));

    anchor_sm.prev_state = ANCHOR_STATE_IDLE;
    anchor_sm.curr_state = ANCHOR_STATE_IDLE;
    anchor_sm.next_state = ANCHOR_STATE_IDLE;
    anchor_sm.timer      = 0;
}

bool anchor_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    anchor_ctx.active = true;
    state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);

    return true;
}

void anchor_stop(void)
{
    anchor_ctx.active          = false;
    anchor_ctx.processing_poll = false;
    anchor_sm.curr_state       = ANCHOR_STATE_IDLE;
    anchor_sm.next_state       = ANCHOR_STATE_IDLE;
}

void anchor_process_1kHz(void)
{
    if (!anchor_ctx.active)
    {
        return;
    }

    // Let state machine handle periodic processing including timeouts
    state_machine_periodic(&anchor_sm);
}

void anchor_set_address(uint16_t address)
{
    uwb_set_address(address, 0xDECA);
}

uint16_t anchor_get_address(void)
{
    return uwb_get_address();
}

void anchor_get_status(anchor_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->state             = (anchor_state_e)anchor_sm.curr_state;
    status->my_address        = uwb_get_address();
    status->polls_received    = anchor_ctx.polls_received;
    status->responses_sent    = anchor_ctx.responses_sent;
    status->response_failures = anchor_ctx.response_failures;
    status->last_tag_address  = anchor_ctx.last_tag_address;
}

uint32_t anchor_get_response_count(void)
{
    return anchor_ctx.responses_sent;
}

void anchor_tx_done_callback(uint32_t message_id, uint64_t tx_timestamp)
{
    if (anchor_sm.curr_state == ANCHOR_STATE_SENDING_RESPONSE)
    {
        // Validate this timestamp belongs to our pending message
        if (message_id == anchor_ctx.pending_tx_id)
        {
            anchor_ctx.resp_tx = tx_timestamp;
            anchor_ctx.responses_sent++;
            anchor_ctx.last_tag_address = anchor_ctx.tag_address;
            anchor_ctx.processing_poll  = false;
            anchor_ctx.pending_tx_id    = 0; // Clear pending ID
            state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_FINAL);
        }
        else
        {
            // Wrong message ID - stale or unexpected callback
            error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                              "TX callback with wrong message ID (got %u, expected %u)",
                              (unsigned int)message_id, (unsigned int)anchor_ctx.pending_tx_id);
            state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
        }
    }
    else if (anchor_sm.curr_state == ANCHOR_STATE_SENDING_FINAL_ACK)
    {
        // FINAL_ACK TX complete - validate message ID and finish transaction
        if (message_id == anchor_ctx.pending_tx_id)
        {
            anchor_ctx.pending_tx_id = 0; // Clear pending ID
            state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
        }
        else
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                              "FINAL_ACK TX callback with wrong message ID");
            state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
        }
    }
    else
    {
        // Unexpected state - log and recover
        error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Unexpected TX callback in state %u",
                          anchor_sm.curr_state);
        anchor_ctx.pending_tx_id = 0;
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
    }
}

void anchor_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr,
                        uint64_t rx_timestamp)
{
    // Check if it's a TWR protocol message
    if (length >= sizeof(protocol_header_t))
    {
        const protocol_header_t* hdr = (const protocol_header_t*)data;

        switch (hdr->msg_type)
        {
            case TWR_MSG_TYPE_POLL:
                // Always accept polls - each poll starts a new ranging transaction
                // This preempts any ongoing transaction (tag handles retries)
                anchor_handle_poll(data, length, src_addr, rx_timestamp);
                break;

            case TWR_MSG_TYPE_FINAL:
                if (anchor_sm.curr_state != ANCHOR_STATE_WAIT_FINAL)
                {
                    error_handler_log(ERROR_SEVERITY_INFO, "anchor",
                                      "FINAL rejected - wrong state (state=%u)",
                                      anchor_sm.curr_state);
                    return;
                }
                if (src_addr != anchor_ctx.tag_address)
                {
                    error_handler_log(ERROR_SEVERITY_INFO, "anchor",
                                      "FINAL rejected - wrong tag (0x%04X vs expected 0x%04X)",
                                      src_addr, anchor_ctx.tag_address);
                    return;
                }
                anchor_handle_final(data, length, src_addr, rx_timestamp);
                break;

            default:
                // Ignore unknown message types
                break;
        }
    }
}