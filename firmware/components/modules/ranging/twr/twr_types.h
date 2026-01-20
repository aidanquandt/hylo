#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DW3000_CLOCK_HZ 499200000ULL
#define DW3000_TIME_UNITS (1.0 / 499.2e6 / 128.0)
#define SPEED_OF_LIGHT 299792458.0
#define TWR_TIMESTAMP_SIZE 5U
#define TWR_RESPONSE_TIMEOUT_MS 500U
#define TWR_PROCESSING_DELAY_US 300U

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
} twr_result_t;

typedef struct
{
    uint64_t timestamp_dtu;
    uint32_t local_time_ms;
} twr_timestamp_t;

/*---------------------------------------------------------------------------
 * Utility Functions
 *---------------------------------------------------------------------------*/
static inline uint64_t twr_timestamp_to_u64(const uint8_t timestamp_bytes[5])
{
    uint64_t ts = 0;
    for (int i = 0; i < 5; i++)
    {
        ts |= ((uint64_t)timestamp_bytes[i]) << (8 * i);
    }
    return ts;
}

static inline void twr_u64_to_timestamp(uint64_t value, uint8_t timestamp_bytes[5])
{
    for (int i = 0; i < 5; i++)
    {
        timestamp_bytes[i] = (uint8_t)(value & 0xFF);
        value >>= 8;
    }
}

static inline int64_t twr_timestamp_diff(uint64_t ts1, uint64_t ts2)
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

static inline float twr_dtu_to_meters(int64_t time_dtu)
{
    return (float)time_dtu * DW3000_TIME_UNITS * SPEED_OF_LIGHT;
}
