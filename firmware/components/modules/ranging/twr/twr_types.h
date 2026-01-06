#pragma once

/*---------------------------------------------------------------------------
 * @file    twr_types.h
 * @brief   Common types and constants for Two-Way Ranging (TWR)
 *---------------------------------------------------------------------------*/

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Constants
 *---------------------------------------------------------------------------*/

// DW3000 timing constants
#define DW3000_CLOCK_HZ 499200000ULL              // 499.2 MHz
#define DW3000_TIME_UNITS (1.0 / 499.2e6 / 128.0) // ~15.65 picoseconds
#define SPEED_OF_LIGHT 299792458.0                // m/s

// Timestamp size
#define TWR_TIMESTAMP_SIZE 5U // 40-bit timestamp

// Timeout values (in milliseconds)
#define TWR_RESPONSE_TIMEOUT_MS 500U // Wait for response from anchor
#define TWR_PROCESSING_DELAY_US 300U // Minimum processing time at anchor

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief TWR status codes
 */
typedef enum
{
    TWR_SUCCESS = 0,
    TWR_ERROR_NULL_PTR,
    TWR_ERROR_INVALID_TIMESTAMP,
    TWR_ERROR_TIMEOUT,
    TWR_ERROR_CALCULATION_FAILED,
    TWR_ERROR_OUT_OF_RANGE
} twr_status_e;

/**
 * @brief TWR ranging result
 */
typedef struct
{
    float distance_m;      // Calculated distance in meters
    float rssi_dbm;        // Received signal strength
    uint16_t remote_addr;  // Address of remote device
    uint32_t timestamp_ms; // Local system time when calculated
    bool valid;            // Result is valid
} twr_result_t;

/**
 * @brief Timestamp storage (40-bit DW3000 timestamp)
 */
typedef struct
{
    uint64_t timestamp_dtu; // Device Time Units (40-bit value)
    uint32_t local_time_ms; // Local system time for timeout tracking
} twr_timestamp_t;

/*---------------------------------------------------------------------------
 * Utility Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Convert 5-byte DW3000 timestamp to uint64_t
 * @param timestamp_bytes 5-byte array from DW3000
 * @return 64-bit timestamp (upper 24 bits will be 0)
 */
static inline uint64_t twr_timestamp_to_u64(const uint8_t timestamp_bytes[5])
{
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++)
    {
        ts |= ((uint64_t)timestamp_bytes[i]) << (8 * i);
    }
    return ts;
}

/**
 * @brief Convert uint64_t to 5-byte DW3000 timestamp
 * @param value 64-bit timestamp (only lower 40 bits used)
 * @param timestamp_bytes Output 5-byte array
 */
static inline void twr_u64_to_timestamp(uint64_t value, uint8_t timestamp_bytes[5])
{
    for (int i = 0; i < 5; i++)
    {
        timestamp_bytes[i] = (uint8_t)(value & 0xFF);
        value >>= 8;
    }
}

/**
 * @brief Calculate difference between two timestamps (handles wraparound)
 * @param ts1 First timestamp
 * @param ts2 Second timestamp
 * @return Difference (ts2 - ts1) in device time units
 */
static inline int64_t twr_timestamp_diff(uint64_t ts1, uint64_t ts2)
{
    // Mask to 40 bits
    uint64_t mask = 0xFFFFFFFFFFULL;
    ts1 &= mask;
    ts2 &= mask;

    int64_t diff = (int64_t)(ts2 - ts1);

    // Handle wraparound (assuming difference is less than half the range)
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

/**
 * @brief Convert device time units to meters
 * @param time_dtu Time in device time units (one-way time of flight)
 * @return Distance in meters
 */
static inline float twr_dtu_to_meters(int64_t time_dtu)
{
    // Distance = time * speed_of_light * time_unit
    // Note: DS-TWR formula already gives one-way ToF, so NO division by 2
    return (float)time_dtu * DW3000_TIME_UNITS * SPEED_OF_LIGHT;
}
