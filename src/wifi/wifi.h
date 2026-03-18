/*---------------------------------------------------------------------------
 * @file    wifi.h
 * @brief   WiFi telemetry module for data transmission
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "sensor_fusion.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

typedef enum
{
    TELEMETRY_EVENT_RANGING,        // Raw ranging measurement
    TELEMETRY_EVENT_SENSOR_EVENT,   // Raw sensor event (IMU or ranging)
    TELEMETRY_EVENT_POSITION,       // Filtered position estimate
    NUM_TELEMETRY_EVENT_TYPES
} telemetry_event_type_e;

typedef enum
{
    WIFI_TELEMETRY_SUCCESS = 0,
    WIFI_TELEMETRY_ERROR_NULL_PTR,
    WIFI_TELEMETRY_ERROR_NOT_INITIALIZED,
    WIFI_TELEMETRY_ERROR_QUEUE_FULL,
    WIFI_TELEMETRY_ERROR_INVALID_TYPE,
    WIFI_TELEMETRY_ERROR_NOT_CONNECTED
} wifi_telemetry_status_e;

// Unified telemetry event structure
typedef struct
{
    telemetry_event_type_e type; // Discriminator for union
    uint32_t timestamp_ms;       // System time when event occurred
    uint32_t sequence;           // Sequence number for debugging

    // Event-specific data
    union
    {
        sensor_ranging_data_t ranging;
        sensor_event_t sensor_event;
        sensor_fusion_position_t position;
    } data;
} telemetry_event_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Push telemetry event to WiFi transmission queue
 * @param event Pointer to telemetry event to send
 * @return Status of the push operation
 */
wifi_telemetry_status_e wifi_push_telemetry(const telemetry_event_t* event);

/**
 * @brief Check if WiFi telemetry is ready to accept data
 * @return true if connected and ready, false otherwise
 */
bool wifi_telemetry_is_ready(void);

/**
 * @brief Send raw protocol bytes over the TCP connection (transparent mode).
 * @param buf Bytes to send (COBS-encoded protocol frame)
 * @param len Length of buf
 */
void wifi_send_protocol_bytes(const uint8_t *buf, size_t len);