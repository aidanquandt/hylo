/*---------------------------------------------------------------------------
 * @file    tag.c
 * @brief   DS-TWR Tag (Initiator) implementation
 * @details Tag sends POLL, receives RESPONSE, sends FINAL with all timestamps
 *---------------------------------------------------------------------------*/

#include "tag.h"
#include "error_handler.h"
#include "platform_os.h"
#include "state_machine.h"
#include "stopwatch.h"
#include "twr/twr_algorithm.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/

// Convert timeout from milliseconds to 1kHz ticks (1 tick = 1ms at 1kHz)
#define TAG_RESPONSE_TIMEOUT_TICKS (TWR_RESPONSE_TIMEOUT_MS)
#define TAG_RETRY_MAX 0U

// Delay from response RX to final TX (in microseconds) for DS-TWR
// Increased to allow for processing overhead
#define RESP_RX_TO_FINAL_TX_DLY_UUS 5000

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/

typedef enum
{
    TAG_FAULT_NONE = 0,
    TAG_FAULT_UWB_NOT_READY,
    TAG_FAULT_SEND_FAILED,
    TAG_FAULT_TIMEOUT,
    TAG_FAULT_INVALID_RESPONSE,
    TAG_FAULT_CALCULATION_FAILED
} tag_fault_e;

typedef struct
{
    bool active;              // Tag is active
    bool ranging_in_progress; // Currently ranging
    uint16_t target_address;  // Current target anchor
    uint16_t sequence;        // Current sequence number
    uint8_t retry_count;      // Retry attempts

    // Timestamps for DS-TWR (all in device time units)
    twr_timestamp_t poll_tx;     // When we sent poll
    twr_timestamp_t resp_rx;     // When we received response
    twr_timestamp_t final_tx;    // When we sent final
    uint64_t poll_rx_ts_remote;  // Anchor's poll RX timestamp
    uint64_t resp_tx_ts_remote;  // Anchor's response TX timestamp
    uint64_t final_rx_ts_remote; // Anchor's final RX timestamp

    // Statistics
    uint32_t successful_ranges;
    uint32_t failed_ranges;
    uint32_t timeout_count;

    // Results
    twr_result_t last_result;
    tag_fault_e fault_code;
} tag_context_t;

typedef struct
{
    bool fault_present;
    bool response_received;  // Response RX callback has been called
    bool final_sent;         // Final message has been sent successfully
    bool final_ack_received; // Final ACK received from anchor
} tag_state_inputs_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

STATIC void tag_state_sample_inputs(void);
STATIC uint16_t tag_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void tag_state_wait_response_process(void);
STATIC void tag_state_wait_final_ack_process(void);
STATIC void tag_state_process_result_on_entry(uint16_t prevState);
STATIC void tag_state_faulted_on_entry(uint16_t prevState);
STATIC bool tag_send_poll(void);
STATIC bool tag_send_final(void);
STATIC void tag_handle_response(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void tag_handle_final_ack(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void tag_calculate_distance(void);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC tag_context_t tag_ctx = {0};
STATIC tag_state_inputs_t tag_inputs = {0};

STATIC const state_s tag_states[] = {
    [TAG_STATE_IDLE] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [TAG_STATE_WAIT_RESPONSE] = {.process = tag_state_wait_response_process,
                                 .onEntry = NULL,
                                 .onExit = NULL},
    [TAG_STATE_WAIT_FINAL_ACK] = {.process = tag_state_wait_final_ack_process,
                                  .onEntry = NULL,
                                  .onExit = NULL},
    [TAG_STATE_PROCESS_RESULT] = {.process = NULL,
                                  .onEntry = tag_state_process_result_on_entry,
                                  .onExit = NULL},
    [TAG_STATE_FAULTED] = {.process = NULL, .onEntry = tag_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s tag_sm = {.prev_state = TAG_STATE_IDLE,
                                 .curr_state = TAG_STATE_IDLE,
                                 .next_state = TAG_STATE_IDLE,
                                 .timer = 0,
                                 .transitionLogic = tag_transition_logic,
                                 .states = tag_states};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void tag_init(void)
{
    memset(&tag_ctx, 0, sizeof(tag_ctx));
    memset(&tag_inputs, 0, sizeof(tag_inputs));

    tag_sm.prev_state = TAG_STATE_IDLE;
    tag_sm.curr_state = TAG_STATE_IDLE;
    tag_sm.next_state = TAG_STATE_IDLE;
    tag_sm.timer = 0;
}

bool tag_start(void)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    tag_ctx.active = true;

    return true;
}

void tag_stop(void)
{
    tag_ctx.active = false;
    tag_ctx.ranging_in_progress = false;

    // Return to idle state
    tag_sm.curr_state = TAG_STATE_IDLE;
    tag_sm.next_state = TAG_STATE_IDLE;
}

void tag_process_1kHz(void)
{
    if (!tag_ctx.active)
    {
        return;
    }

    tag_state_sample_inputs();
    state_machine_periodic(&tag_sm);
}

bool tag_start_ranging(uint16_t anchor_addr)
{
    stopwatch_start();
    if (!tag_ctx.active)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Not active");
        return false;
    }

    if (tag_ctx.ranging_in_progress)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Ranging already in progress");
        return false;
    }

    if (!uwb_is_ready())
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "UWB not ready");
        return false;
    }

    // Initialize ranging context
    tag_ctx.ranging_in_progress = true;
    tag_ctx.target_address = anchor_addr;
    tag_ctx.sequence++;
    tag_ctx.retry_count = 0;
    tag_ctx.fault_code = TAG_FAULT_NONE;

    // Clear timestamps
    memset(&tag_ctx.poll_tx, 0, sizeof(tag_ctx.poll_tx));
    memset(&tag_ctx.resp_rx, 0, sizeof(tag_ctx.resp_rx));
    memset(&tag_ctx.final_tx, 0, sizeof(tag_ctx.final_tx));
    tag_ctx.poll_rx_ts_remote = 0;
    tag_ctx.resp_tx_ts_remote = 0;
    tag_ctx.final_rx_ts_remote = 0;

    // Clear state input flags
    tag_inputs.response_received = false;
    tag_inputs.final_ack_received = false;
    tag_inputs.fault_present = false;

    // Reset state machine timer
    tag_sm.timer = 0;

    // Send poll immediately (synchronous operation)
    if (!tag_send_poll())
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Failed to send poll");
        tag_ctx.fault_code = TAG_FAULT_SEND_FAILED;
        tag_ctx.ranging_in_progress = false;
        return false;
    }

    // Transition directly to wait for response state
    tag_sm.next_state = TAG_STATE_WAIT_RESPONSE;

    return true;
}

bool tag_is_ranging(void)
{
    return tag_ctx.ranging_in_progress;
}

bool tag_get_last_result(twr_result_t* result)
{
    if (result == NULL)
    {
        return false;
    }

    if (!tag_ctx.last_result.valid)
    {
        return false;
    }

    *result = tag_ctx.last_result;
    return true;
}

void tag_get_status(tag_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->state = (tag_state_e)tag_sm.curr_state;
    status->target_address = tag_ctx.target_address;
    status->successful_ranges = tag_ctx.successful_ranges;
    status->failed_ranges = tag_ctx.failed_ranges;
    status->timeout_count = tag_ctx.timeout_count;
    status->last_result = tag_ctx.last_result;
}

void tag_cancel_ranging(void)
{
    tag_ctx.ranging_in_progress = false;
    tag_sm.next_state = TAG_STATE_IDLE;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - State Machine
 *---------------------------------------------------------------------------*/

STATIC void tag_state_sample_inputs(void)
{
    tag_inputs.fault_present = (tag_ctx.fault_code != TAG_FAULT_NONE);
    // NOTE: response_received and final_sent are set by RX callback and need to persist
    // until state transition logic checks them
}

STATIC uint16_t tag_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;

    switch (currentState)
    {
        case TAG_STATE_IDLE:
            // Transitions handled externally via tag_start_ranging()
            break;

        case TAG_STATE_WAIT_RESPONSE:
            if (tag_inputs.fault_present)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "tag", "  -> FAULTED (fault_present)\r\n");
                nextState = TAG_STATE_FAULTED;
            }
            else if (tag_inputs.response_received)
            {
                // Send final message and transition to wait for ACK
                if (!tag_send_final())
                {
                    error_handler_log(ERROR_SEVERITY_ERROR, "tag", "Failed to send final");
                    tag_ctx.fault_code = TAG_FAULT_SEND_FAILED;
                    nextState = TAG_STATE_FAULTED;
                }
                else
                {
                    tag_inputs.response_received = false;
                    nextState = TAG_STATE_WAIT_FINAL_ACK;
                }
            }
            else if (stateTimer >= TAG_RESPONSE_TIMEOUT_TICKS)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "tag", "  -> FAULTED (timeout)\r\n");
                tag_ctx.fault_code = TAG_FAULT_TIMEOUT;
                nextState = TAG_STATE_FAULTED;
            }
            break;

        case TAG_STATE_WAIT_FINAL_ACK:
            // Wait for FINAL_ACK or timeout
            if (tag_inputs.final_ack_received)
            {
                nextState = TAG_STATE_PROCESS_RESULT;
            }
            else if (stateTimer >= TAG_RESPONSE_TIMEOUT_TICKS)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "tag", "  -> FAULTED (ACK timeout)\r\n");
                tag_ctx.fault_code = TAG_FAULT_TIMEOUT;
                nextState = TAG_STATE_FAULTED;
            }
            break;

        case TAG_STATE_PROCESS_RESULT:
            // Always return to idle after processing
            nextState = TAG_STATE_IDLE;
            break;

        case TAG_STATE_FAULTED:
            // Stay in faulted or retry
            nextState = TAG_STATE_IDLE;
            break;

        default:
            nextState = TAG_STATE_IDLE;
            break;
    }

    return nextState;
}

STATIC void tag_state_wait_response_process(void)
{
    // Waiting is passive - response handled in RX callback
}

STATIC void tag_state_wait_final_ack_process(void)
{
    // Waiting for FINAL_ACK from anchor
    // Timeout handled by state machine transition logic
}

STATIC void tag_state_process_result_on_entry(uint16_t prevState)
{
    (void)prevState;

    tag_calculate_distance();
    tag_ctx.ranging_in_progress = false;

    // Clear flags now that we've processed them
    tag_inputs.response_received = false;
    tag_inputs.final_ack_received = false;
}

STATIC void tag_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    tag_ctx.failed_ranges++;

    if (tag_ctx.fault_code == TAG_FAULT_TIMEOUT)
    {
        tag_ctx.timeout_count++;
    }

    // Clear flags since we're aborting this attempt
    tag_inputs.response_received = false;

    // Retry logic
    if (tag_ctx.retry_count < TAG_RETRY_MAX)
    {
        error_handler_log(ERROR_SEVERITY_INFO, "tag", "Retrying (attempt %u/%u)",
                          tag_ctx.retry_count + 1, TAG_RETRY_MAX);

        tag_ctx.retry_count++;
        tag_ctx.fault_code = TAG_FAULT_NONE;

        // Reset timer for WAIT_RESPONSE state
        tag_sm.timer = 0;

        // Resend poll and transition to wait for response
        if (tag_send_poll())
        {
            tag_sm.next_state = TAG_STATE_WAIT_RESPONSE;
        }
        else
        {
            // Send failed, will stay in FAULTED and try again next cycle
            tag_ctx.fault_code = TAG_FAULT_SEND_FAILED;
        }
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Max retries exceeded");
        tag_ctx.ranging_in_progress = false;
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Ranging failed after %u retries",
                          TAG_RETRY_MAX);
    }
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - TWR Protocol
 *---------------------------------------------------------------------------*/

STATIC bool tag_send_poll(void)
{
    protocol_twr_poll_msg_t poll;
    poll.header.protocol_type = PROTOCOL_TYPE_TWR;
    poll.header.msg_type = TWR_MSG_TYPE_POLL;
    poll.header.sequence = tag_ctx.sequence;

    // Send poll message
    if (!uwb_send_message((uint8_t*)&poll, sizeof(poll), tag_ctx.target_address))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "POLL TX failed");
        return false;
    }

    // Read TX timestamp (automatically captured by hardware)
    tag_ctx.poll_tx.timestamp_dtu = uwb_get_last_tx_timestamp();
    tag_ctx.poll_tx.local_time_ms = platform_os_gettick();

    return true;
}

STATIC bool tag_send_final(void)
{
    protocol_twr_final_msg_t final;
    final.header.protocol_type = PROTOCOL_TYPE_TWR;
    final.header.msg_type = TWR_MSG_TYPE_FINAL;
    final.header.sequence = tag_ctx.sequence;

    // Convert response RX timestamp to 5-byte format
    twr_u64_to_timestamp(tag_ctx.resp_rx.timestamp_dtu, final.resp_rx_ts);

    // Calculate the scheduled final TX time (full 40-bit DTU)
    // Formula: TX_time = RX_time + response_delay
    // Antenna delay is handled by hardware configuration
    // (dwt_setrxantennadelay/dwt_settxantennadelay)
    uint64_t delay_dtu = UWB_US_TO_DTU(RESP_RX_TO_FINAL_TX_DLY_UUS);

    // Full 40-bit absolute TX time in DTU - mask to 40 bits to handle wraparound
    uint64_t final_tx_time_dtu = (tag_ctx.resp_rx.timestamp_dtu + delay_dtu) & 0xFFFFFFFFFFULL;

    // Extract high 32-bits for delayed TX API (hardware requires DTUH format)
    uint32_t final_tx_time_dtuh = UWB_DTU_TO_DTUH(final_tx_time_dtu);

    // Encode the scheduled TX timestamp in the final message
    twr_u64_to_timestamp(final_tx_time_dtu, final.final_tx_ts);

    // Send final message with delayed TX (pass DTUH - high 32-bits)
    if (!uwb_send_message_delayed((uint8_t*)&final, sizeof(final), tag_ctx.target_address,
                                  final_tx_time_dtuh))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "FINAL TX failed");
        return false;
    }

    // Read actual TX timestamp from hardware
    tag_ctx.final_tx.timestamp_dtu = uwb_get_last_tx_timestamp();
    if (tag_ctx.final_tx.timestamp_dtu == 0)
    {
        tag_ctx.final_tx.timestamp_dtu = final_tx_time_dtu; // Fallback to scheduled
    }
    tag_ctx.final_tx.local_time_ms = platform_os_gettick();

    return true;
}

void tag_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    // Check if response is from our target
    if (src_addr != tag_ctx.target_address)
    {
        error_handler_log(ERROR_SEVERITY_INFO, "tag",
                          "REJECTED: wrong source (0x%04X vs expected 0x%04X)", src_addr,
                          tag_ctx.target_address);
        return;
    }

    // Route messages based on current state and message type
    if (length >= sizeof(protocol_header_t))
    {
        const protocol_header_t* header = (const protocol_header_t*)data;

        // In WAIT_RESPONSE state, process RESPONSE messages
        if (tag_sm.curr_state == TAG_STATE_WAIT_RESPONSE &&
            header->msg_type == TWR_MSG_TYPE_RESPONSE)
        {
            tag_handle_response(data, length, src_addr);
        }
        // In WAIT_FINAL_ACK state, process FINAL_ACK messages
        else if (tag_sm.curr_state == TAG_STATE_WAIT_FINAL_ACK &&
                 header->msg_type == TWR_MSG_TYPE_FINAL_ACK)
        {
            tag_handle_final_ack(data, length, src_addr);
        }
        else
        {
            error_handler_log(ERROR_SEVERITY_INFO, "tag",
                              "REJECTED: state=%u, msg_type=0x%02X mismatch", tag_sm.curr_state,
                              header->msg_type);
        }
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_INFO, "tag", "REJECTED: message too short for header");
    }
}

STATIC void tag_handle_response(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    (void)src_addr;

    // Validate message
    if (length < sizeof(protocol_twr_response_msg_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "Response REJECT - too short (%u < %u)",
                          length, sizeof(protocol_twr_response_msg_t));
        tag_ctx.fault_code = TAG_FAULT_INVALID_RESPONSE;
        return;
    }

    const protocol_twr_response_msg_t* resp = (const protocol_twr_response_msg_t*)data;

    if (resp->header.msg_type != TWR_MSG_TYPE_RESPONSE || resp->header.sequence != tag_ctx.sequence)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag",
                          "Response REJECT - msg_type=0x%02X (expect 0x%02X), seq=%u (expect %u)",
                          resp->header.msg_type, TWR_MSG_TYPE_RESPONSE, resp->header.sequence,
                          tag_ctx.sequence);
        tag_ctx.fault_code = TAG_FAULT_INVALID_RESPONSE;
        return;
    }

    // Record RX timestamp (get from UWB radio)
    tag_ctx.resp_rx.local_time_ms = platform_os_gettick();
    tag_ctx.resp_rx.timestamp_dtu = uwb_get_last_rx_timestamp();

    // Extract remote poll RX timestamp from response
    tag_ctx.poll_rx_ts_remote = twr_timestamp_to_u64(resp->poll_rx_ts);

    // Note: resp_tx_ts will arrive in FINAL_ACK after anchor measures actual TX time

    // Signal that we received response (triggers SEND_FINAL state)
    tag_inputs.response_received = true;
}

STATIC void tag_handle_final_ack(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    (void)src_addr;

    // Validate message
    if (length < sizeof(protocol_twr_final_ack_msg_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag", "FINAL_ACK REJECT - too short (%u < %u)",
                          length, sizeof(protocol_twr_final_ack_msg_t));
        tag_ctx.fault_code = TAG_FAULT_INVALID_RESPONSE;
        return;
    }

    const protocol_twr_final_ack_msg_t* ack = (const protocol_twr_final_ack_msg_t*)data;

    if (ack->header.msg_type != TWR_MSG_TYPE_FINAL_ACK || ack->header.sequence != tag_ctx.sequence)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "tag",
                          "FINAL_ACK REJECT - msg_type=0x%02X (expect 0x%02X), seq=%u (expect %u)",
                          ack->header.msg_type, TWR_MSG_TYPE_FINAL_ACK, ack->header.sequence,
                          tag_ctx.sequence);
        tag_ctx.fault_code = TAG_FAULT_INVALID_RESPONSE;
        return;
    }

    // Extract anchor's response TX timestamp from the ACK (now included)
    tag_ctx.resp_tx_ts_remote = twr_timestamp_to_u64(ack->resp_tx_ts);

    // Extract anchor's final RX timestamp from the ACK
    tag_ctx.final_rx_ts_remote = twr_timestamp_to_u64(ack->final_rx_ts);

    // Signal that we received final ACK (triggers PROCESS_RESULT state)
    tag_inputs.final_ack_received = true;
}

STATIC void tag_calculate_distance(void)
{
    // Calculate distance using DS-TWR with all 6 timestamps
    twr_result_t result = {0};

    // Use DS-TWR formula with 6 timestamps
    twr_status_e status = twr_calculate_ds_twr(tag_ctx.poll_tx.timestamp_dtu, // Poll TX (tag)
                                               tag_ctx.poll_rx_ts_remote,     // Poll RX (anchor)
                                               tag_ctx.resp_tx_ts_remote, // Response TX (anchor)
                                               tag_ctx.resp_rx.timestamp_dtu,  // Response RX (tag)
                                               tag_ctx.final_tx.timestamp_dtu, // Final TX (tag)
                                               tag_ctx.final_rx_ts_remote,     // Final RX (anchor)
                                               &result);

    if (status == TWR_SUCCESS && result.valid)
    {
        stopwatch_stop();
        uint32_t elapsed_us = stopwatch_elapsed_us();
        uart_manager_print("DS-TWR SUCCESS: distance=%.2f m, time=%lu us\r\n", result.distance_m,
                           elapsed_us);
        result.remote_addr = tag_ctx.target_address;
        result.timestamp_ms = platform_os_gettick();

        tag_ctx.last_result = result;
        tag_ctx.successful_ranges++;
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "tag", "DS-TWR calculation failed: %d", status);
        tag_ctx.fault_code = TAG_FAULT_CALCULATION_FAILED;
        tag_ctx.failed_ranges++;
    }
}
