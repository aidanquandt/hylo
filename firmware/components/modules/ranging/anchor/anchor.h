#pragma once

/*---------------------------------------------------------------------------
 * @file    anchor.h
 * @brief   SS-TWR Anchor (Responder) state machine
 * @details Implements the anchor/responder side of Single-Sided TWR ranging
 *          (2-message protocol: receives POLL, sends RESPONSE with timestamps)
 *---------------------------------------------------------------------------*/

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Anchor state machine states
 */
typedef enum
{
    ANCHOR_STATE_IDLE,          // Not active
    ANCHOR_STATE_LISTENING,     // Listening for polls
    ANCHOR_STATE_SEND_RESPONSE, // Sending response
    ANCHOR_STATE_FAULTED        // Error state
} anchor_state_e;

/**
 * @brief Anchor status information
 */
typedef struct
{
    anchor_state_e state;       // Current state
    uint16_t my_address;        // Our anchor address
    uint32_t polls_received;    // Total polls received
    uint32_t responses_sent;    // Total responses sent
    uint32_t response_failures; // Failed responses
    uint16_t last_tag_address;  // Last tag we responded to
} anchor_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize anchor module
 */
void anchor_init(void);

/**
 * @brief Start anchor (enable responding to ranging requests)
 * @return true on success
 */
bool anchor_start(void);

/**
 * @brief Stop anchor (disable responding)
 */
void anchor_stop(void);

/**
 * @brief Periodic processing (called at 1kHz)
 */
void anchor_process_1kHz(void);

/**
 * @brief Set anchor address
 * @param address Our anchor address
 */
void anchor_set_address(uint16_t address);

/**
 * @brief Get anchor address
 * @return Current anchor address
 */
uint16_t anchor_get_address(void);

/**
 * @brief Get anchor status
 * @param status Output: Anchor status information
 */
void anchor_get_status(anchor_status_t* status);

/**
 * @brief Get number of responses sent
 * @return Total responses sent
 */
uint32_t anchor_get_response_count(void);
/**
 * @brief Internal RX callback for protocol router
 * @param data Message data
 * @param length Message length
 * @param src_addr Source address
 * @note Called by ranging module protocol handler
 */
void anchor_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr);