#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "uwb_protocol_messages.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DW3000_CLOCK_HZ 499200000ULL
#define DW3000_TIME_UNITS (1.0 / 499.2e6 / 128.0)
#define SPEED_OF_LIGHT 299792458.0

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    TWR_SUCCESS = 0,
    TWR_ERROR_NULL_PTR,
    TWR_ERROR_INVALID_TIMESTAMP,
    TWR_ERROR_TIMEOUT,
    TWR_ERROR_CALCULATION_FAILED,
    TWR_ERROR_OUT_OF_RANGE
} twr_status_e;

typedef struct
{
    float distance_m;
    float rssi_dbm;
    uint16_t remote_addr;
    uint32_t timestamp_ms;
    bool valid;
    // Anchor position (from anchor's response)
    vec3_t anchor_position;
    bool anchor_position_valid;
} twr_result_t;

typedef struct
{
    uint64_t timestamp_dtu;
    uint32_t local_time_ms;
} twr_timestamp_t;

/*---------------------------------------------------------------------------
 * Utility Functions
 *---------------------------------------------------------------------------*/
STATIC inline uint64_t twr_timestamp_to_u64(const uint8_t timestamp_bytes[5])
{
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++)
    {
        ts |= ((uint64_t)timestamp_bytes[i]) << (8 * i);
    }
    return ts;
}

STATIC inline void twr_u64_to_timestamp(uint64_t value, uint8_t timestamp_bytes[5])
{
    for (int i = 0; i < 5; i++)
    {
        timestamp_bytes[i] = (uint8_t)(value & 0xFF);
        value >>= 8;
    }
}

STATIC inline int64_t twr_timestamp_diff(uint64_t ts1, uint64_t ts2)
{
    uint64_t mask = 0xFFFFFFFFFFULL;
    ts1 &= mask;
    ts2 &= mask;

    int64_t diff = (int64_t)(ts2 - ts1);

    int64_t half_range = (int64_t)(mask >> 1);
    if (diff > half_range)
    {
        diff -= (int64_t)(mask + 1);
    }
    else if (diff < -half_range)
    {
        diff += (int64_t)(mask + 1);
    }

    return diff;
}

STATIC inline float twr_dtu_to_meters(int64_t time_dtu)
{
    return (float)time_dtu * DW3000_TIME_UNITS * SPEED_OF_LIGHT;
}

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
// Generic TWR states that work for both initiator and responder
typedef enum
{
    TWR_STATE_IDLE,      // Ready for next transaction
    TWR_STATE_SENDING,   // Transmitting a message
    TWR_STATE_WAITING,   // Waiting for response/next message
    TWR_STATE_PROCESSING // Processing results/preparing next action
} twr_state_e;

// TWR roles (protocol-level)
typedef enum
{
    TWR_ROLE_INITIATOR, // Initiates ranging transaction
    TWR_ROLE_RESPONDER  // Responds to ranging requests
} twr_role_e;

// TWR fault codes
typedef enum
{
    TWR_FAULT_NONE = 0,
    TWR_FAULT_UWB_NOT_READY,
    TWR_FAULT_SEND_FAILED,
    TWR_FAULT_TIMEOUT,
    TWR_FAULT_INVALID_MESSAGE,
    TWR_FAULT_CALCULATION_FAILED
} twr_fault_e;

// Unified event system
typedef enum
{
    TWR_EVENT_TX_COMPLETE,
    TWR_EVENT_RX_MESSAGE,
    TWR_EVENT_TIMEOUT,
    TWR_EVENT_FAULT
} twr_event_type_e;

typedef struct
{
    twr_event_type_e type;
    union
    {
        struct
        {
            uint32_t message_id;
            uint64_t tx_timestamp;
            twr_msg_type_e msg_type;
        } tx;
        struct
        {
            const uint8_t* data;
            uint16_t length;
            uint16_t src_addr;
            uint64_t rx_timestamp;
        } rx;
        struct
        {
            uint32_t elapsed_ms;
            twr_msg_type_e expected_msg;
        } timeout;
        struct
        {
            twr_fault_e fault_code;
            const char* description;
        } fault;
    };
} twr_event_t;

// Forward declaration
struct twr_context_s;

// Role-specific behavior callbacks
typedef struct
{
    bool (*send_message)(twr_msg_type_e msg_type, struct twr_context_s* ctx);
    void (*handle_message)(const twr_event_t* event, struct twr_context_s* ctx);
    void (*handle_tx_complete)(const twr_event_t* event, struct twr_context_s* ctx);
    void (*handle_timeout)(const twr_event_t* event, struct twr_context_s* ctx);
    void (*handle_fault)(const twr_event_t* event, struct twr_context_s* ctx);
    void (*process_result)(struct twr_context_s* ctx);
} twr_callbacks_t;

// Unified TWR context
typedef struct twr_context_s
{
    // State machine
    twr_state_e state;
    twr_role_e role;
    uint32_t state_timer;

    // Current transaction
    twr_msg_type_e expected_msg;   // What we're waiting for
    twr_msg_type_e pending_tx_msg; // What we're currently sending
    uint16_t sequence;
    uint16_t peer_address; // Initiator: responder addr, Responder: current initiator addr
    uint32_t pending_tx_id;
    uint8_t retry_count;

    // Timestamps
    twr_timestamp_t timestamps[4]; // poll_tx, resp_rx, final_tx, (unused for now)
    uint64_t remote_timestamps[4]; // poll_rx, resp_tx, final_rx, (unused)

    // Anchor position (received from responder messages)
    vec3_t remote_anchor_position;
    bool remote_anchor_position_valid;

    // Statistics
    uint32_t successful_transactions;
    uint32_t failed_transactions;
    uint32_t timeout_count;

    // Results and state
    twr_result_t last_result;
    twr_fault_e fault_code;

    // Role-specific callbacks
    const twr_callbacks_t* callbacks;
    void* role_context; // Points to tag_context_t or anchor_context_t
} twr_context_t;
