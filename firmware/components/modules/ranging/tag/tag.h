#pragma once

/*---------------------------------------------------------------------------
 * @file    tag.h
 * @brief   DS-TWR Tag (Initiator) state machine
 * @details Implements the tag/initiator side of Double-Sided TWR ranging
 *          (3-message protocol: POLL -> RESPONSE -> FINAL)
 *---------------------------------------------------------------------------*/

#include "common.h"
#include "twr/twr_types.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Tag state machine states for DS-TWR
 */
typedef enum
{
    TAG_STATE_IDLE,           // Not ranging
    TAG_STATE_WAIT_RESPONSE,  // Waiting for anchor response
    TAG_STATE_SEND_FINAL,     // Sending final message with timestamps
    TAG_STATE_WAIT_FINAL_ACK, // Waiting for final ACK with anchor timestamps
    TAG_STATE_PROCESS_RESULT, // Calculating distance
    TAG_STATE_FAULTED         // Error state
} tag_state_e;

/**
 * @brief Tag status information
 */
typedef struct
{
    tag_state_e state;          // Current state
    uint16_t target_address;    // Address we're ranging to
    uint32_t successful_ranges; // Total successful ranges
    uint32_t failed_ranges;     // Total failed ranges
    uint32_t timeout_count;     // Number of timeouts
    twr_result_t last_result;   // Last ranging result
} tag_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize tag module
 */
void tag_init(void);

/**
 * @brief Start tag (enable ranging)
 * @return true on success
 */
bool tag_start(void);

/**
 * @brief Stop tag (disable ranging)
 */
void tag_stop(void);

/**
 * @brief Periodic processing (called at 1kHz)
 */
void tag_process_1kHz(void);

/**
 * @brief Initiate ranging to specific anchor
 * @param anchor_addr Address of anchor to range to
 * @return true if ranging started, false if already ranging or error
 */
bool tag_start_ranging(uint16_t anchor_addr);

/**
 * @brief Check if ranging is in progress
 * @return true if actively ranging
 */
bool tag_is_ranging(void);

/**
 * @brief Get last ranging result
 * @param result Output: Last ranging result
 * @return true if valid result available
 */
bool tag_get_last_result(twr_result_t* result);

/**
 * @brief Get tag status
 * @param status Output: Tag status information
 */
void tag_get_status(tag_status_t* status);

/**
 * @brief Cancel current ranging operation
 */
void tag_cancel_ranging(void);

/**
 * @brief Internal RX callback for protocol router
 * @param data Message data
 * @param length Message length
 * @param src_addr Source address
 * @note Called by ranging module protocol handler
 */
void tag_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr);
