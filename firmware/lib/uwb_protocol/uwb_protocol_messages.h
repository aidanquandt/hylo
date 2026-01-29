#pragma once

/*---------------------------------------------------------------------------
 * @file    uwb_protocol_messages.h
 * @brief   UWB message protocol structures and definitions
 * @details Defines common message header and protocol types for all
 *          application-layer messages over UWB radio
 *---------------------------------------------------------------------------*/

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Protocol Type Codes (Top-Level)
 *---------------------------------------------------------------------------*/
typedef enum
{
    PROTOCOL_TYPE_TWR  = 0x01, // Two-Way Ranging protocol
    PROTOCOL_TYPE_DATA = 0x02  // Data exchange protocol
    // Reserve 0x03-0x0F for future protocols
} protocol_type_e;

/*---------------------------------------------------------------------------
 * TWR Sub-Message Types
 *---------------------------------------------------------------------------*/
typedef enum
{
    TWR_MSG_POLL      = 0x01, // Tag -> Anchor: Ranging request (initiates DS-TWR)
    TWR_MSG_RESPONSE  = 0x02, // Anchor -> Tag: Response with timestamps
    TWR_MSG_FINAL     = 0x03, // Tag -> Anchor: Final message with all timestamps
    TWR_MSG_FINAL_ACK = 0x04  // Anchor -> Tag: ACK with anchor's final RX timestamp
} twr_msg_type_e;

/*---------------------------------------------------------------------------
 * DATA Sub-Message Types
 *---------------------------------------------------------------------------*/
typedef enum
{
    DATA_MSG_BEACON    = 0x01, // Simple beacon/ping
    DATA_MSG_TELEMETRY = 0x02, // Sensor data
    DATA_MSG_COMMAND   = 0x03  // Command/control
    // Reserve 0x04-0xFF for future data types
} data_msg_type_e;

/*---------------------------------------------------------------------------
 * Common Message Header
 *---------------------------------------------------------------------------*/

/**
 * @brief Common header for all protocol messages
 * @note All application messages MUST start with this header
 *
 * Architecture: This header sits INSIDE the MAC payload. Complete frame structure:
 *   [MAC Header (9B)] + [Protocol Header (4B)] + [Protocol Payload]
 *
 * The protocol sequence field is used for transaction correlation (e.g., matching
 * POLL → RESPONSE → FINAL in TWR), while MAC sequence is link-layer only.
 */
typedef struct
{
    uint8_t protocol_type; // Top-level protocol (PROTOCOL_TYPE_*)
    uint8_t msg_type;      // Protocol-specific message type
    uint16_t sequence;     // Transaction sequence for protocol-level correlation
} __attribute__((packed)) protocol_header_t;

/*---------------------------------------------------------------------------
 * Protocol Message Structures
 *---------------------------------------------------------------------------*/

/**
 * @brief TWR Poll message (Tag -> Anchor)
 */
typedef struct
{
    protocol_header_t header; // protocol_type = PROTOCOL_TYPE_TWR, msg_type = TWR_MSG_POLL
} __attribute__((packed)) protocol_twr_poll_msg_t;

/**
 * @brief TWR Response message (Anchor -> Tag)
 * Used in DS-TWR to send back timestamps from anchor
 */
typedef struct
{
    protocol_header_t header; // protocol_type = PROTOCOL_TYPE_TWR, msg_type = TWR_MSG_RESPONSE
    uint8_t poll_rx_ts[5];    // When anchor received poll (40-bit timestamp)
    vec3_t anchor_position;   // Anchor position in meters (zeros if unknown)
    // Note: resp_tx_ts is sent later in FINAL_ACK after actual transmission
} __attribute__((packed)) protocol_twr_response_msg_t;

/**
 * @brief TWR Final message (Tag -> Anchor)
 * Completes DS-TWR exchange by sending tag's timestamps back to anchor
 */
typedef struct
{
    protocol_header_t header; // protocol_type = PROTOCOL_TYPE_TWR, msg_type = TWR_MSG_FINAL
    uint8_t resp_rx_ts[5];    // When tag received response (40-bit timestamp)
    uint8_t final_tx_ts[5];   // When tag will send final (40-bit timestamp, pre-calculated)
} __attribute__((packed)) protocol_twr_final_msg_t;

/**
 * @brief TWR Final ACK message (Anchor -> Tag)
 * Acknowledges receipt of FINAL message with anchor's timestamps for distance calculation
 */
typedef struct
{
    protocol_header_t header; // protocol_type = PROTOCOL_TYPE_TWR, msg_type = TWR_MSG_FINAL_ACK
    uint8_t resp_tx_ts[5];    // When anchor actually transmitted response (40-bit timestamp)
    uint8_t final_rx_ts[5];   // When anchor received final message (40-bit timestamp)
    vec3_t anchor_position;   // Anchor position in meters (zeros if unknown)
} __attribute__((packed)) protocol_twr_final_ack_msg_t;

/**
 * @brief Data Beacon message (simple ping/counter)
 */
typedef struct
{
    protocol_header_t header; // protocol_type = PROTOCOL_TYPE_DATA, msg_type = DATA_MSG_TYPE_BEACON
    uint16_t counter;         // Auto-incrementing counter or custom data
    uint8_t payload[];        // Variable length payload (optional)
} __attribute__((packed)) protocol_data_beacon_msg_t;

/*---------------------------------------------------------------------------
 * Message Size Constraints
 *---------------------------------------------------------------------------*/

#define PROTOCOL_HEADER_SIZE sizeof(protocol_header_t)
#define PROTOCOL_MIN_MESSAGE_SIZE PROTOCOL_HEADER_SIZE
#define PROTOCOL_MAX_PAYLOAD_SIZE (125U - PROTOCOL_HEADER_SIZE) // 802.15.4 max - header

/*---------------------------------------------------------------------------
 * Helper Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Check if protocol type is valid
 * @param protocol_type Protocol type code
 * @return true if valid protocol type
 */
static inline bool uwb_protocol_is_valid_type(uint8_t protocol_type)
{
    return (protocol_type == PROTOCOL_TYPE_TWR || protocol_type == PROTOCOL_TYPE_DATA);
}

/**
 * @brief Extract protocol type from raw message
 * @param data Message data
 * @param length Message length
 * @return Protocol type, or 0xFF if invalid
 */
static inline uint8_t uwb_protocol_get_type(const uint8_t* data, uint16_t length)
{
    if (data == NULL || length < PROTOCOL_HEADER_SIZE)
    {
        return 0xFF;
    }
    return data[0];
}

/**
 * @brief Extract message type from raw message
 * @param data Message data
 * @param length Message length
 * @return Message type, or 0xFF if invalid
 */
static inline uint8_t uwb_protocol_get_msg_type(const uint8_t* data, uint16_t length)
{
    if (data == NULL || length < PROTOCOL_HEADER_SIZE)
    {
        return 0xFF;
    }
    return data[1];
}

/**
 * @brief Extract sequence number from raw message
 * @param data Message data
 * @param length Message length
 * @return Sequence number, or 0 if invalid
 */
static inline uint16_t uwb_protocol_get_sequence(const uint8_t* data, uint16_t length)
{
    if (data == NULL || length < PROTOCOL_HEADER_SIZE)
    {
        return 0;
    }
    const protocol_header_t* hdr = (const protocol_header_t*)data;
    return hdr->sequence;
}
