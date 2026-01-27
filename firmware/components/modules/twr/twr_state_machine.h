#pragma once

/*---------------------------------------------------------------------------
 * @file    twr_state_machine.h
 * @brief   Unified TWR state machine for both tag and anchor
 * @details Generic state machine with role-specific callback system
 *---------------------------------------------------------------------------*/

#include "twr_types.h"

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/
#define TWR_MAX_RETRIES 2U
#define TWR_TIMEOUT_MS 50U

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize unified TWR context
 */
void twr_init(twr_context_t* ctx, twr_role_e role, const twr_callbacks_t* callbacks,
              void* role_context);

/**
 * @brief Start TWR operation (tag initiates ranging, anchor starts listening)
 */
bool twr_start(twr_context_t* ctx, uint16_t peer_addr);

/**
 * @brief Stop TWR operation
 */
void twr_stop(twr_context_t* ctx);

/**
 * @brief Process state machine (call at 1kHz)
 */
void twr_process(twr_context_t* ctx);

/**
 * @brief Handle unified TWR event
 */
void twr_handle_event(twr_context_t* ctx, const twr_event_t* event);

/**
 * @brief Generic UWB RX callback - converts to TWR event
 */
void twr_rx_callback(twr_context_t* ctx, const uint8_t* data, uint16_t length, uint16_t src_addr,
                     uint64_t rx_timestamp);

/**
 * @brief Generic UWB TX callback - converts to TWR event
 */
void twr_tx_done_callback(twr_context_t* ctx, uint32_t message_id, uint64_t tx_timestamp);

/**
 * @brief Get current state
 */
twr_state_e twr_get_state(const twr_context_t* ctx);

/**
 * @brief Check if transaction is active
 */
bool twr_is_active(const twr_context_t* ctx);

/**
 * @brief Cancel current transaction
 */
void twr_cancel(twr_context_t* ctx);

/*---------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Transition to new state (for use by role implementations)
 */
void twr_transition_to(twr_context_t* ctx, twr_state_e new_state);