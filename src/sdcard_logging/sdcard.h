/*---------------------------------------------------------------------------
 * @file    sdcard.h
 * @brief   SD card logging module - writes UWB, IMU, and position data to CSV
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
    SDCARD_EVENT_RANGING,   // UWB ranging measurement
    SDCARD_EVENT_IMU,       // IMU sample
    SDCARD_EVENT_POSITION,  // Sensor fusion position estimate
    NUM_SDCARD_EVENT_TYPES
} sdcard_event_type_e;

typedef enum
{
    SDCARD_PUSH_SUCCESS = 0,
    SDCARD_PUSH_ERROR_NULL_PTR,
    SDCARD_PUSH_ERROR_NOT_INITIALIZED,
    SDCARD_PUSH_ERROR_QUEUE_FULL,
    SDCARD_PUSH_ERROR_INVALID_TYPE,
} sdcard_push_status_e;

typedef struct
{
    sdcard_event_type_e type;
    uint32_t            timestamp_ms;
    union
    {
        sensor_ranging_data_t  ranging;
        sensor_imu_data_t      imu;
        sensor_fusion_position_t position;
    } data;
} sdcard_log_event_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Push a log event into the SD card write queue (non-blocking).
 * @return SDCARD_PUSH_SUCCESS if enqueued; error code otherwise.
 */
sdcard_push_status_e sdcard_push_event(const sdcard_log_event_t* event);

/**
 * @brief Returns true when the SD card is mounted and the log file is open.
 */
bool sdcard_is_logging(void);