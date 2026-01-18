/*---------------------------------------------------------------------------
 * @file    anchor.c
 * @brief   DS-TWR Anchor (Responder) implementation
 * @details Anchor receives POLL and responds with pre-calculated timestamps using delayed TX
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

// Delay from poll RX to response TX (in microseconds)
// Increased to allow for processing overhead
#define POLL_RX_TO_RESP_TX_DLY_UUS 5000

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

typedef struct
{
    bool fault_present;
    bool poll_received;
} anchor_state_inputs_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

STATIC void anchor_state_sample_inputs(void);
STATIC uint16_t anchor_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void anchor_state_listening_on_entry(uint16_t prevState);
STATIC void anchor_state_faulted_on_entry(uint16_t prevState);
STATIC void anchor_handle_poll(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void anchor_handle_final(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC bool anchor_send_response(void);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC anchor_context_t anchor_ctx = {0};
STATIC anchor_state_inputs_t anchor_inputs = {0};

STATIC const state_s anchor_states[] = {
    [ANCHOR_STATE_IDLE] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [ANCHOR_STATE_LISTENING] = {.process = NULL,
                                .onEntry = anchor_state_listening_on_entry,
                                .onExit = NULL},
    [ANCHOR_STATE_FAULTED] = {
        .process = NULL, .onEntry = anchor_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s anchor_sm = {.prev_state = ANCHOR_STATE_IDLE,
                                    .curr_state = ANCHOR_STATE_IDLE,
                                    .next_state = ANCHOR_STATE_IDLE,
                                    .timer = 0,
                                    .transitionLogic = anchor_transition_logic,
                                    .states = anchor_states};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void anchor_init(void)
{
    memset(&anchor_ctx, 0, sizeof(anchor_ctx));
    memset(&anchor_inputs, 0, sizeof(anchor_inputs));

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

    // Start listening
    anchor_sm.next_state = ANCHOR_STATE_LISTENING;

    return true;
}

void anchor_stop(void)
{
    anchor_ctx.active = false;
    anchor_ctx.processing_poll = false;

    // Return to idle
    anchor_sm.curr_state = ANCHOR_STATE_IDLE;
    anchor_sm.next_state = ANCHOR_STATE_IDLE;
}

void anchor_process_1kHz(void)
{
    if (!anchor_ctx.active)
    {
        return;
    }

    anchor_state_sample_inputs();
    state_machine_periodic(&anchor_sm);
}

void anchor_set_address(uint16_t address)
{
    // Just forward to UWB layer
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

STATIC void anchor_state_sample_inputs(void)
{
    anchor_inputs.fault_present = (anchor_ctx.fault_code != ANCHOR_FAULT_NONE);
}

STATIC uint16_t anchor_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    (void)stateTimer; // Unused - polling happens in RX callback now

    switch (currentState)
    {
        case ANCHOR_STATE_IDLE:
            // External trigger to start listening
            break;

        case ANCHOR_STATE_LISTENING:
            if (anchor_inputs.fault_present)
            {
                nextState = ANCHOR_STATE_FAULTED;
            }
            break;

        case ANCHOR_STATE_FAULTED:
            // Return to listening after fault
            nextState = ANCHOR_STATE_LISTENING;
            break;

        default:
            nextState = ANCHOR_STATE_IDLE;
            break;
    }

    return nextState;
}

STATIC void anchor_state_listening_on_entry(uint16_t prevState)
{
    (void)prevState;

    // Clear fault from previous attempt
    anchor_ctx.fault_code = ANCHOR_FAULT_NONE;
}

STATIC void anchor_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    error_handler_log(ERROR_SEVERITY_WARNING, "anchor", "Fault: %d", anchor_ctx.fault_code);

    // Clear fault for next attempt
    anchor_ctx.fault_code = ANCHOR_FAULT_NONE;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - TWR Protocol
 *---------------------------------------------------------------------------*/

void anchor_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    // Check if it's a TWR protocol message
    if (length >= sizeof(protocol_header_t))
    {
        const protocol_header_t* hdr = (const protocol_header_t*)data;

        // Handle POLL messages when listening
        if (hdr->protocol_type == PROTOCOL_TYPE_TWR && hdr->msg_type == TWR_MSG_TYPE_POLL)
        {
            if (anchor_sm.curr_state != ANCHOR_STATE_LISTENING || anchor_ctx.processing_poll)
            {
                uart_manager_print("Anchor RX: POLL rejected (state=%u, processing=%u)\r\n",
                                   anchor_sm.curr_state, anchor_ctx.processing_poll);
                return;
            }
            anchor_handle_poll(data, length, src_addr);
        }
        // Handle FINAL messages at any time
        else if (hdr->protocol_type == PROTOCOL_TYPE_TWR && hdr->msg_type == TWR_MSG_TYPE_FINAL)
        {
            anchor_handle_final(data, length, src_addr);
        }
    }
}

STATIC void anchor_handle_poll(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    // Validate message
    if (length < sizeof(protocol_twr_poll_msg_t))
    {
        anchor_ctx.fault_code = ANCHOR_FAULT_INVALID_POLL;
        uart_manager_print("Anchor: POLL validation FAILED - too short\r\n");
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

    // Record RX timestamp from UWB radio
    uwb_dev_t* uwb_dev = uwb_get_device();
    if (uwb_dev == NULL)
    {
        anchor_ctx.poll_rx.timestamp_dtu = 0;
        uart_manager_print("Anchor: Failed to get UWB device\r\n");
        anchor_ctx.fault_code = ANCHOR_FAULT_UWB_NOT_READY;
        anchor_ctx.processing_poll = false;
        return;
    }

    anchor_ctx.poll_rx.timestamp_dtu = uwb_get_last_rx_timestamp();

    anchor_ctx.polls_received++;

    // Send response immediately (no state machine delay)
    if (!anchor_send_response())
    {
        anchor_ctx.fault_code = ANCHOR_FAULT_SEND_FAILED;
        anchor_ctx.response_failures++;
        uart_manager_print("Anchor: POLL handling FAILED - response send error\r\n");
    }
    else
    {
        anchor_ctx.responses_sent++;
        anchor_ctx.last_tag_address = anchor_ctx.tag_address;
    }

    anchor_ctx.processing_poll = false;
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
        uart_manager_print("Anchor: RESPONSE delayed TX FAILED\r\n");
        return false;
    }

    // Capture actual TX timestamp from hardware after delayed transmission
    anchor_ctx.resp_tx = uwb_get_last_tx_timestamp();
    if (anchor_ctx.resp_tx == 0)
    {
        anchor_ctx.resp_tx = absolute_tx_time_dtu; // Fallback to scheduled time
    }

    return true;
}

STATIC void anchor_handle_final(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    // Validate message
    if (length < sizeof(protocol_twr_final_msg_t))
    {
        uart_manager_print("FINAL REJECT - too short (%u < %u)\r\n", length,
                           sizeof(protocol_twr_final_msg_t));
        anchor_ctx.fault_code = ANCHOR_FAULT_INVALID_POLL;
        return;
    }

    const protocol_twr_final_msg_t* final = (const protocol_twr_final_msg_t*)data;

    // Record RX timestamp for the FINAL message (get from UWB radio)
    uint64_t final_rx_ts_dtu = uwb_get_last_rx_timestamp();
    if (final_rx_ts_dtu == 0)
    {
        uart_manager_print("anchor_handle_final: Failed to get RX timestamp\r\n");
        anchor_ctx.fault_code = ANCHOR_FAULT_SEND_FAILED;
        return;
    }

    // Send FINAL_ACK with anchor's measured RESPONSE TX timestamp and FINAL RX timestamp
    protocol_twr_final_ack_msg_t ack;
    ack.header.protocol_type = PROTOCOL_TYPE_TWR;
    ack.header.msg_type = TWR_MSG_TYPE_FINAL_ACK;
    ack.header.sequence = final->header.sequence;

    // Encode anchor's response TX timestamp (measured from earlier) in the ACK message
    twr_u64_to_timestamp(anchor_ctx.resp_tx, ack.resp_tx_ts);

    // Encode anchor's final RX timestamp in the ACK message
    twr_u64_to_timestamp(final_rx_ts_dtu, ack.final_rx_ts);

    // Send ACK immediately (no delayed TX needed for ACK)
    if (!uwb_send_message((uint8_t*)&ack, sizeof(ack), src_addr))
    {
        uart_manager_print("anchor_handle_final: Failed to send ACK to 0x%04X\r\n", src_addr);
        anchor_ctx.fault_code = ANCHOR_FAULT_SEND_FAILED;
        return;
    }
}
