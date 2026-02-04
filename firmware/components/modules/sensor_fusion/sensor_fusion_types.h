#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SENSOR_FUSION_QUEUE_SIZE 32U

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

// Sensor event types
typedef enum
{
    SENSOR_EVENT_RANGING,     // UWB TWR ranging measurement
    SENSOR_EVENT_IMU,         // IMU sample (accel + gyro)
    SENSOR_EVENT_MAGNETOMETER // Future: magnetometer (compass)
} sensor_event_type_e;

// Status codes for sensor fusion operations
typedef enum
{
    SENSOR_FUSION_SUCCESS = 0,
    SENSOR_FUSION_ERROR_NULL_PTR,
    SENSOR_FUSION_ERROR_QUEUE_FULL,
    SENSOR_FUSION_ERROR_QUEUE_EMPTY,
    SENSOR_FUSION_ERROR_INVALID_TYPE
} sensor_fusion_status_e;

// Ranging event data (from UWB TWR)
typedef struct
{
    float distance_m;           // Measured distance in meters
    float rssi_dbm;             // Signal strength
    vec3_t anchor_position;     // Position of anchor in 3D space (m) - needed for trilateration
    bool anchor_position_valid; // Whether anchor position is known
    uint16_t anchor_addr;       // Anchor address (for debugging/logging only)
    float quality;              // Measurement quality/confidence [0.0 - 1.0]
} sensor_ranging_data_t;

// IMU event data (accelerometer + gyroscope)
typedef struct
{
    float accel_x; // Acceleration X-axis (m/s²)
    float accel_y; // Acceleration Y-axis (m/s²)
    float accel_z; // Acceleration Z-axis (m/s²)
    float gyro_x;  // Angular velocity X-axis (rad/s)
    float gyro_y;  // Angular velocity Y-axis (rad/s)
    float gyro_z;  // Angular velocity Z-axis (rad/s)
    float temp_c;  // Temperature (°C)
} sensor_imu_data_t;

// Magnetometer event data (future use)
typedef struct
{
    float mag_x; // Magnetic field X-axis (µT)
    float mag_y; // Magnetic field Y-axis (µT)
    float mag_z; // Magnetic field Z-axis (µT)
} sensor_mag_data_t;

// Unified sensor event structure (self-contained, no pointers)
typedef struct
{
    sensor_event_type_e type; // Discriminator for union
    uint32_t timestamp_ms;    // System time when event occurred
    uint32_t sequence;        // Sequence number for debugging

    // Event-specific data
    union
    {
        sensor_ranging_data_t ranging;
        sensor_imu_data_t imu;
        sensor_mag_data_t mag;
    } data;
} sensor_event_t;

// Queue statistics for health monitoring
typedef struct
{
    uint32_t events_pushed;     // Total events added
    uint32_t events_popped;     // Total events consumed
    uint32_t overflows;         // Times queue was full (oldest dropped)
    uint32_t current_depth;     // Current number of items in queue
    uint32_t max_depth_reached; // Maximum depth ever reached
} sensor_queue_stats_t;

// Position estimate output (updated by fusion task)
typedef struct
{
    float x;               // Position X (meters)
    float y;               // Position Y (meters)
    float z;               // Position Z (meters)
    float vx;              // Velocity X (m/s)
    float vy;              // Velocity Y (m/s)
    float vz;              // Velocity Z (m/s)
    float confidence;      // Estimate confidence [0.0 - 1.0]
    uint32_t timestamp_ms; // When estimate was computed
    bool valid;            // Whether estimate is valid
} sensor_fusion_position_t;
