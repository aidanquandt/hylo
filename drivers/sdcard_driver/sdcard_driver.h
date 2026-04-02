#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stddef.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    SDCARD_DRIVER_EVENT_RANGING = 0,
    SDCARD_DRIVER_EVENT_IMU,
    SDCARD_DRIVER_EVENT_POSITION,
    NUM_SDCARD_DRIVER_EVENT_TYPES
} sdcard_driver_event_type_E;

typedef struct
{
    float distance_m;
    uint16_t anchor_addr;
    float anchor_x;
    float anchor_y;
    float anchor_z;
    float quality;
    float rssi_dbm;
} sdcard_driver_ranging_event_t;

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp_c;
} sdcard_driver_imu_event_t;

typedef struct
{
    float x;
    float y;
    float z;
    float vx;
    float vy;
    float vz;
    float confidence;
    bool valid;
    int imu_enable;
    float roll;
    float pitch;
    float yaw;
} sdcard_driver_position_event_t;

typedef struct
{
    sdcard_driver_event_type_E type;
    uint32_t timestamp_ms;
    union
    {
        sdcard_driver_ranging_event_t ranging;
        sdcard_driver_imu_event_t imu;
        sdcard_driver_position_event_t position;
    } data;
} sdcard_driver_event_t;

typedef enum
{
    SDCARD_DRIVER_PUSH_SUCCESS = 0,
    SDCARD_DRIVER_PUSH_ERROR_NULL_PTR,
    SDCARD_DRIVER_PUSH_ERROR_NOT_INITIALIZED,
    SDCARD_DRIVER_PUSH_ERROR_QUEUE_FULL,
    SDCARD_DRIVER_PUSH_ERROR_INVALID_TYPE,
} sdcard_driver_push_status_E;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void sdcard_driver_init(void);
sdcard_driver_push_status_E sdcard_driver_push_event(const sdcard_driver_event_t* event);
bool sdcard_driver_is_logging(void);