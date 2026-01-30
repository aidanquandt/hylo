/*---------------------------------------------------------------------------
 * @file    mac_802154.h
 * @brief   IEEE 802.15.4 MAC frame definitions and utilities
 * @note    Minimal implementation for UWB applications using short addressing
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdint.h>

/*---------------------------------------------------------------------------
 * 802.15.4 Frame Control Field Values
 *---------------------------------------------------------------------------*/
// Frame type (bits 0-2)
typedef enum
{
    MAC_FC_TYPE_BEACON  = 0x0000, // Beacon frame
    MAC_FC_TYPE_DATA    = 0x0001, // Data frame
    MAC_FC_TYPE_ACK     = 0x0002, // Acknowledgment frame
    MAC_FC_TYPE_COMMAND = 0x0003  // MAC command frame
} mac_frame_type_e;

// Security (bit 3)
typedef enum
{
    MAC_FC_SEC_DISABLED = 0x0000, // No security
    MAC_FC_SEC_ENABLED  = 0x0008  // Security enabled
} mac_security_e;

// Frame pending (bit 4)
typedef enum
{
    MAC_FC_FRAME_PEND_NO  = 0x0000, // No frame pending
    MAC_FC_FRAME_PEND_YES = 0x0010  // More data pending
} mac_frame_pending_e;

// Acknowledgment request (bit 5)
typedef enum
{
    MAC_FC_ACK_REQ_NO  = 0x0000, // No ACK requested
    MAC_FC_ACK_REQ_YES = 0x0020  // ACK requested
} mac_ack_request_e;

// PAN ID compression (bit 6)
typedef enum
{
    MAC_FC_PANID_COMP_NO  = 0x0000, // No PAN ID compression
    MAC_FC_PANID_COMP_YES = 0x0040  // PAN ID compression
} mac_panid_compression_e;

// Destination addressing mode (bits 10-11)
typedef enum
{
    MAC_FC_DST_ADDR_NONE  = 0x0000, // No destination address
    MAC_FC_DST_ADDR_SHORT = 0x0800, // 16-bit destination address
    MAC_FC_DST_ADDR_LONG  = 0x0C00  // 64-bit destination address
} mac_dst_addr_mode_e;

// Frame version (bits 12-13)
typedef enum
{
    MAC_FC_FRAME_VER_2003 = 0x0000, // IEEE 802.15.4-2003
    MAC_FC_FRAME_VER_2006 = 0x1000  // IEEE 802.15.4-2006
} mac_frame_version_e;

// Source addressing mode (bits 14-15)
typedef enum
{
    MAC_FC_SRC_ADDR_NONE  = 0x0000, // No source address
    MAC_FC_SRC_ADDR_SHORT = 0x8000, // 16-bit source address
    MAC_FC_SRC_ADDR_LONG  = 0xC000  // 64-bit source address
} mac_src_addr_mode_e;

/*---------------------------------------------------------------------------
 * Common MAC Constants
 *---------------------------------------------------------------------------*/
#define MAC_DEFAULT_PAN_ID (0xDECAU) // Default PAN ID
#define MAC_BROADCAST_ADDR (0xFFFF)  // Broadcast address
#define MAC_MAX_FRAME_SIZE (127U)    // Maximum 802.15.4 frame size

/*---------------------------------------------------------------------------
 * 802.15.4 MAC Frame Structures
 *---------------------------------------------------------------------------*/

/**
 * @brief 802.15.4 MAC frame with short addressing (16-bit addresses)
 * @note This is the most common format for UWB/ranging applications
 *
 * Design Note: MAC sequence field is auto-incremented but not used for
 * transaction correlation. Protocol-layer sequence (in protocol_header_t)
 * handles message correlation since we operate without MAC-layer ACKs.
 */
typedef struct __attribute__((packed))
{
    uint16_t frame_control; // Frame control field
    uint8_t sequence;       // MAC sequence (auto-incremented, not actively used)
    uint16_t dest_pan_id;   // Destination PAN ID
    uint16_t dest_addr;     // Destination address (16-bit)
    uint16_t src_addr;      // Source address (16-bit)
    uint8_t payload[];      // Application protocol messages (protocol_header_t + data)
} mac_frame_short_t;

/*---------------------------------------------------------------------------
 * Helper Macros
 *---------------------------------------------------------------------------*/

/** Size of MAC header with short addressing (bytes) */
#define MAC_FRAME_SHORT_HEADER_SIZE (sizeof(mac_frame_short_t))

/** Maximum payload size with short addressing */
#define MAC_MAX_PAYLOAD_SIZE (MAC_MAX_FRAME_SIZE - MAC_FRAME_SHORT_HEADER_SIZE)

/**
 * @brief Build frame control field from components
 * @note OR together the MAC_FC_* defines to build the complete field
 *
 * Example:
 *   frame.frame_control = MAC_BUILD_FC(
 *       MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT
 *   );
 */
#define MAC_BUILD_FC(...) (__VA_ARGS__)
