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
// Delay from POLL RX to RESPONSE TX (microseconds)
// Conservative: 1000µs allows plenty of processing time with event-driven architecture
// Aggressive: Can go as low as ~600µs (matching DW3000 examples)
#define POLL_RX_TO_RESP_TX_DLY_UUS 800
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
    bool processing_poll; // Currently processing a poll
    uint16_t tag_address; // Tag we're responding to
    uint16_t sequence;    // Current sequence number

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
    [ANCHOR_STATE_IDLE] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_WAIT_POLL] = {.process = NULL,
                                .onEntry = anchor_state_wait_poll_on_entry,
                                .onExit = NULL},
    [ANCHOR_STATE_SENDING_RESPONSE] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_WAIT_FINAL] = {.process = NULL,
                                 .onEntry = anchor_state_wait_final_on_entry,
                                 .onExit = NULL},
    [ANCHOR_STATE_SENDING_FINAL_ACK] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_FAULTED] = {
        .process = NULL, .onEntry = anchor_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s anchor_sm = {.prev_state = ANCHOR_STATE_IDLE,
                                    .curr_state = ANCHOR_STATE_IDLE,
                                    .next_state = ANCHOR_STATE_IDLE,
                                    .timer = 0,
                                    .transitionLogic = NULL, // Event-driven - no transition logic
                                    .states = anchor_states};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void anchor_init(void)
{
    memset(&anchor_ctx, 0, sizeof(anchor_ctx));

    anchor_sm.prev_state = ANCHOR_STATE_IDLE;
    anchor_sm.curr_state = ANCHOR_STATE_IDLE;
    anchor_sm.next_state = ANCHOR_STATE_IDLE;
    anchor_sm.timer = 0;
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
    anchor_ctx.active = false;
    anchor_ctx.processing_poll = false;
    anchor_sm.curr_state = ANCHOR_STATE_IDLE;
    anchor_sm.next_state = ANCHOR_STATE_IDLE;
}

void anchor_process_1kHz(void)
{
    if (!anchor_ctx.active)
    {
        return;
    }

    // Check for timeout waiting for FINAL message
    if (anchor_sm.curr_state == ANCHOR_STATE_WAIT_FINAL)
    {
        if (anchor_sm.timer < UINT32_MAX)
        {
            anchor_sm.timer++;
        }

        if (anchor_sm.timer >= ANCHOR_FINAL_TIMEOUT_TICKS)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "anchor",
                              "Timeout waiting for FINAL from tag 0x%04X", anchor_ctx.tag_address);
            // Return to WAIT_POLL and ready for next tag
            state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
        }
    }
    else if (anchor_sm.curr_state == ANCHOR_STATE_FAULTED)
    {
        // Auto-recover from faults by returning to WAIT_POLL
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
    }
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

    status->state = (anchor_state_e)anchor_sm.curr_state;
    status->my_address = uwb_get_address();
    status->polls_received = anchor_ctx.polls_received;
    status->responses_sent = anchor_ctx.responses_sent;
    status->response_failures = anchor_ctx.response_failures;
    status->last_tag_address = anchor_ctx.last_tag_address;
}

uint32_t anchor_get_response_count(void)
{
    return anchor_ctx.responses_sent;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - State Machine
 *---------------------------------------------------------------------------*/

STATIC void anchor_state_wait_poll_on_entry(uint16_t prevState)
{
    (void)prevState;

    // Clear any previous transaction context
    anchor_ctx.processing_poll = false;
    anchor_ctx.tag_address = 0;
    anchor_ctx.sequence = 0;
    anchor_ctx.fault_code = ANCHOR_FAULT_NONE;
}

STATIC void anchor_state_wait_final_on_entry(uint16_t prevState)
{
    (void)prevState;

    // Reset timer for timeout detection
    anchor_sm.timer = 0;
}

STATIC void anchor_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Fault: %d", anchor_ctx.fault_code);

    // Clear fault for next attempt
    anchor_ctx.fault_code = ANCHOR_FAULT_NONE;

    // Will auto-recover to WAIT_POLL in process_1kHz
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - TWR Protocol
 *---------------------------------------------------------------------------*/

void anchor_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr,
                        uint64_t rx_timestamp)
{
    // Check if it's a TWR protocol message
    if (length >= sizeof(protocol_header_t))
    {
        const protocol_header_t* hdr = (const protocol_header_t*)data;

        // Handle POLL messages only in WAIT_POLL state
        if (hdr->protocol_type == PROTOCOL_TYPE_TWR && hdr->msg_type == TWR_MSG_TYPE_POLL)
        {
            if (anchor_sm.curr_state != ANCHOR_STATE_WAIT_POLL)
            {
                error_handler_log(ERROR_SEVERITY_INFO, "anchor",
                                  "POLL rejected - wrong state (state=%u)", anchor_sm.curr_state);
                return;
            }
            anchor_handle_poll(data, length, src_addr, rx_timestamp);
        }
        // Handle FINAL messages only in WAIT_FINAL state from the expected tag
        else if (hdr->protocol_type == PROTOCOL_TYPE_TWR && hdr->msg_type == TWR_MSG_TYPE_FINAL)
        {
            if (anchor_sm.curr_state != ANCHOR_STATE_WAIT_FINAL)
            {
                error_handler_log(ERROR_SEVERITY_INFO, "anchor",
                                  "FINAL rejected - wrong state (state=%u)", anchor_sm.curr_state);
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
        }
    }
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
    anchor_ctx.tag_address = src_addr;
    anchor_ctx.sequence = poll->header.sequence;
    anchor_ctx.processing_poll = true;

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
    response.header.msg_type = TWR_MSG_TYPE_RESPONSE;
    response.header.sequence = anchor_ctx.sequence;

    // Store poll RX timestamp in response (in 5-byte format for transmission)
    // This is the only timestamp we know with certainty at this point
    twr_u64_to_timestamp(anchor_ctx.poll_rx.timestamp_dtu, response.poll_rx_ts);

    // Calculate absolute TX time (full 40-bit DTU) for delayed transmission
    // Formula: TX_time = RX_time + response_delay
    // Antenna delay is handled by hardware configuration
    // (dwt_setrxantennadelay/dwt_settxantennadelay)
    uint64_t delay_dtu = UWB_US_TO_DTU(POLL_RX_TO_RESP_TX_DLY_UUS);

    // Full 40-bit absolute TX time in DTU - mask to 40 bits to handle wraparound
    uint64_t absolute_tx_time_dtu =
        (anchor_ctx.poll_rx.timestamp_dtu + delay_dtu) & 0xFFFFFFFFFFULL;

    // Extract high 32-bits for delayed TX API (hardware requires DTUH format)
    uint32_t absolute_tx_time_dtuh = UWB_DTU_TO_DTUH(absolute_tx_time_dtu);

    // Send response with delayed transmission (pass DTUH - high 32-bits)
    if (!uwb_send_message_delayed((uint8_t*)&response, sizeof(response), anchor_ctx.tag_address,
                                  absolute_tx_time_dtuh))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "RESPONSE delayed TX FAILED");
        return false;
    }

    // Store the scheduled time as fallback
    anchor_ctx.resp_tx = absolute_tx_time_dtu;

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
    ack.header.msg_type = TWR_MSG_TYPE_FINAL_ACK;
    ack.header.sequence = sequence;

    // Encode anchor's response TX timestamp (measured from earlier) in the ACK message
    twr_u64_to_timestamp(resp_tx_ts, ack.resp_tx_ts);

    // Encode anchor's final RX timestamp in the ACK message
    twr_u64_to_timestamp(final_rx_ts, ack.final_rx_ts);

    // Send ACK immediately (no delayed TX needed for ACK)
    if (!uwb_send_message((uint8_t*)&ack, sizeof(ack), tag_addr))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "anchor", "Failed to send ACK to 0x%04X", tag_addr);
        return false;
    }

    return true;
}

void anchor_tx_done_callback(uint64_t tx_timestamp)
{
    // Use state machine state to determine which TX completed
    if (anchor_sm.curr_state == ANCHOR_STATE_SENDING_RESPONSE)
    {
        // Capture RESPONSE TX timestamp
        anchor_ctx.resp_tx = tx_timestamp;

        // Update statistics
        anchor_ctx.responses_sent++;
        anchor_ctx.last_tag_address = anchor_ctx.tag_address;
        anchor_ctx.processing_poll = false;

        // Transition to WAIT_FINAL
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_FINAL);
    }
    else if (anchor_sm.curr_state == ANCHOR_STATE_SENDING_FINAL_ACK)
    {
        // FINAL_ACK TX complete, transaction finished
        // Return to WAIT_POLL for next tag
        state_machine_force_transition(&anchor_sm, ANCHOR_STATE_WAIT_POLL);
    }
}
