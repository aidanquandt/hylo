#pragma once

/*---------------------------------------------------------------------------
 * @file    sensor_fusion.h
 * @brief   Error-State Kalman Filter (ESKF) for IMU + UWB TWR fusion
 * @details Fuses IMU (accelerometer/gyroscope) with UWB Two-Way Ranging
 *          measurements from 4 anchors to estimate position, velocity,
 *          and attitude.
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ESKF_NUM_ANCHORS 4U
#define ESKF_STATE_DIM 6U   // Position (3) + Velocity (3)
#define ESKF_ERROR_DIM 9U   // Position (3) + Velocity (3) + Attitude error (3)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

/**
 * @brief 3D vector
 */
typedef struct
{
    float x;
    float y;
    float z;
} eskf_vec3_t;

/**
 * @brief Quaternion [w, x, y, z]
 */
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} eskf_quat_t;

/**
 * @brief Anchor position configuration
 */
typedef struct
{
    eskf_vec3_t position[ESKF_NUM_ANCHORS]; ///< Anchor positions in world frame
    bool configured;                         ///< Whether anchors are configured
} eskf_anchor_config_t;

/**
 * @brief IMU measurement input
 */
typedef struct
{
    eskf_vec3_t accel;  ///< Accelerometer [m/s^2]
    eskf_vec3_t gyro;   ///< Gyroscope [rad/s]
} eskf_imu_meas_t;

/**
 * @brief TWR range measurement
 */
typedef struct
{
    uint8_t anchor_id;  ///< Anchor ID (0-3)
    float range_m;      ///< Measured range in meters
    bool valid;         ///< Measurement validity
} eskf_range_meas_t;

/**
 * @brief ESKF state output
 */
typedef struct
{
    eskf_vec3_t position;   ///< Position [m]
    eskf_vec3_t velocity;   ///< Velocity [m/s]
    eskf_quat_t attitude;   ///< Attitude quaternion (body to world)
    float position_std[3];  ///< Position standard deviation [m]
    float velocity_std[3];  ///< Velocity standard deviation [m/s]
    bool initialized;       ///< Filter is initialized
} eskf_state_t;

/**
 * @brief ESKF statistics
 */
typedef struct
{
    uint32_t imu_updates;       ///< Total IMU prediction steps
    uint32_t uwb_updates;       ///< Total UWB correction steps
    uint32_t uwb_rejected;      ///< Rejected UWB measurements (outliers)
} eskf_stats_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Configure anchor positions
 * @param anchors Array of 4 anchor positions [x, y, z] in meters
 */
void sensor_fusion_set_anchors(const eskf_vec3_t anchors[ESKF_NUM_ANCHORS]);

/**
 * @brief Set initial position estimate
 * @param position Initial position [x, y, z] in meters
 */
void sensor_fusion_set_initial_position(const eskf_vec3_t* position);

/**
 * @brief Process IMU measurement (prediction step)
 * @param imu IMU measurement (accel in m/s^2, gyro in rad/s)
 * @param dt Time step in seconds
 */
void sensor_fusion_imu_update(const eskf_imu_meas_t* imu, float dt);

/**
 * @brief Process UWB TWR range measurement (correction step)
 * @param range Range measurement from one anchor
 */
void sensor_fusion_uwb_update(const eskf_range_meas_t* range);

/**
 * @brief Get current state estimate
 * @param state Output state estimate
 */
void sensor_fusion_get_state(eskf_state_t* state);

/**
 * @brief Get filter statistics
 * @param stats Output statistics
 */
void sensor_fusion_get_stats(eskf_stats_t* stats);

/**
 * @brief Reset the filter to initial state
 */
void sensor_fusion_reset(void);

/**
 * @brief Check if filter is initialized and running
 * @return true if filter is initialized
 */
bool sensor_fusion_is_initialized(void);

/**
 * @brief Legacy ISR function (placeholder)
 */
void sensor_fusion_isr(void);
