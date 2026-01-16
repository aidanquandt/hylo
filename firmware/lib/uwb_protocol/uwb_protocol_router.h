#pragma once

/*---------------------------------------------------------------------------
 * @file    uwb_protocol_router.h
 * @brief   UWB message router/dispatcher
 * @details Central router that receives all UWB messages and dispatches
 *          to appropriate protocol handlers based on message type
 *---------------------------------------------------------------------------*/

#include "uwb_protocol_messages.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Protocol handler callback function
 * @param data Pointer to message data (including header)
 * @param length Total message length
 * @param src_addr Source address (from MAC layer)
 */
typedef void (*protocol_handler_t)(const uint8_t* data, uint16_t length, uint16_t src_addr);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize protocol router
 * @note Does not register with UWB - call uwb_protocol_router_rx_callback from your UWB RX handler
 */
void uwb_protocol_router_init(void);

/**
 * @brief Main RX callback - call this from UWB RX callback
 * @param data Pointer to received message data
 * @param length Message length
 * @param src_addr Source address (from MAC layer)
 */
void uwb_protocol_router_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr);

/**
 * @brief Register a protocol handler
 * @param protocol_type Protocol type (PROTOCOL_TYPE_*)
 * @param handler Handler function to call for this protocol
 * @return true if registered successfully
 */
bool uwb_protocol_router_register_handler(uint8_t protocol_type, protocol_handler_t handler);

/**
 * @brief Unregister a protocol handler
 * @param protocol_type Protocol type to unregister
 */
void uwb_protocol_router_unregister_handler(uint8_t protocol_type);

/**
 * @brief Get router statistics
 * @param total_received Output: Total messages received
 * @param unhandled Output: Messages with no registered handler
 * @param invalid Output: Invalid/malformed messages
 */
void uwb_protocol_router_get_stats(uint32_t* total_received, uint32_t* unhandled,
                                   uint32_t* invalid);

/**
 * @brief Reset router statistics
 */
void uwb_protocol_router_reset_stats(void);
