/**
 * @file sensor_fusion.h
 * @brief Sensor Fusion Module - Public API
 *
 * This module provides IMU/UWB sensor fusion using an Extended Kalman Filter
 * for 3D position, velocity, and attitude estimation.
 *
 * Usage:
 *   1. Call sensor_fusion_init() once at startup
 *   2. Call sensor_fusion_set_anchor_position() to configure anchor locations
 *   3. Call sensor_fusion_predict_imu() with each IMU sample (~100-1000 Hz)
 *   4. Call sensor_fusion_update_twr() with each UWB range measurement (~10-100 Hz)
 *   5. Call sensor_fusion_get_state() to retrieve the current estimated state
 *
 * The filter automatically handles timing, process noise, and state finalization.
 *
 * Adapted from Crazyflie firmware (GPLv3) for STM32H7 platform.
 */

#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

/** Maximum number of UWB anchors supported */
#define SF_MAX_ANCHORS 8

/** Number of default anchors */
#define SF_DEFAULT_NUM_ANCHORS 4

/*---------------------------------------------------------------------------
 * Type definitions
 *---------------------------------------------------------------------------*/

/**
 * @brief 3D vector type
 */
typedef struct {
    float x;
    float y;
    float z;
} sf_vector3_t;

/**
 * @brief Quaternion type [w, x, y, z]
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} sf_quaternion_t;

/**
 * @brief Euler angles in radians
 */
typedef struct {
    float roll;   /**< Roll angle (rad) */
    float pitch;  /**< Pitch angle (rad) */
    float yaw;    /**< Yaw angle (rad) */
} sf_euler_t;

/**
 * @brief Complete state estimate from sensor fusion
 */
typedef struct {
    sf_vector3_t position;     /**< Position in world frame (m) */
    sf_vector3_t velocity;     /**< Velocity in world frame (m/s) */
    sf_euler_t attitude;       /**< Attitude as Euler angles (rad) */
    sf_quaternion_t quaternion; /**< Attitude as quaternion */
    uint32_t timestamp_ms;     /**< Timestamp of last update (ms) */
    bool valid;                /**< True if estimate is valid */
} sensor_fusion_state_t;

/**
 * @brief UWB anchor position configuration
 */
typedef struct {
    float x;       /**< Anchor X position in world frame (m) */
    float y;       /**< Anchor Y position in world frame (m) */
    float z;       /**< Anchor Z position in world frame (m) */
    bool enabled;  /**< True if anchor is active */
} sf_anchor_config_t;

/**
 * @brief Sensor fusion configuration parameters
 */
typedef struct {
    /* Initial position (m) */
    float initial_x;
    float initial_y;
    float initial_z;
    float initial_yaw;  /**< Initial yaw angle (rad) */
    
    /* Process noise tuning */
    float proc_noise_acc_xy;   /**< Accelerometer XY process noise */
    float proc_noise_acc_z;    /**< Accelerometer Z process noise */
    
    /* UWB measurement noise */
    float uwb_std_dev;         /**< Default UWB measurement std dev (m) */
    
    /* Use robust M-estimation for UWB updates */
    bool use_robust_twr;
} sensor_fusion_config_t;

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the sensor fusion module with default configuration
 *
 * Must be called before any other sensor_fusion functions.
 * Uses default anchor positions and filter parameters.
 */
void sensor_fusion_init(void);

/**
 * @brief Initialize the sensor fusion module with custom configuration
 *
 * @param config Pointer to configuration structure
 */
void sensor_fusion_init_with_config(const sensor_fusion_config_t* config);

/**
 * @brief Reset the filter to initial state
 *
 * Useful for recovering from divergence or reinitializing after
 * the device has been moved to a new location.
 */
void sensor_fusion_reset(void);

/**
 * @brief Set the position of a UWB anchor
 *
 * @param anchor_id Anchor index (0 to SF_MAX_ANCHORS-1)
 * @param x X position in world frame (m)
 * @param y Y position in world frame (m)
 * @param z Z position in world frame (m)
 * @return true if successful, false if anchor_id is invalid
 */
bool sensor_fusion_set_anchor_position(uint8_t anchor_id, float x, float y, float z);

/**
 * @brief Enable or disable an anchor
 *
 * @param anchor_id Anchor index (0 to SF_MAX_ANCHORS-1)
 * @param enabled True to enable, false to disable
 * @return true if successful, false if anchor_id is invalid
 */
bool sensor_fusion_set_anchor_enabled(uint8_t anchor_id, bool enabled);

/**
 * @brief Prediction step using IMU data
 *
 * Call this function each time new IMU data is available.
 * Typically at the IMU sample rate (100-1000 Hz).
 *
 * @param ax Accelerometer X reading (m/s², body frame)
 * @param ay Accelerometer Y reading (m/s², body frame)
 * @param az Accelerometer Z reading (m/s², body frame)
 * @param gx Gyroscope X reading (rad/s, body frame)
 * @param gy Gyroscope Y reading (rad/s, body frame)
 * @param gz Gyroscope Z reading (rad/s, body frame)
 * @param timestamp_ms Current time in milliseconds
 */
void sensor_fusion_predict_imu(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint32_t timestamp_ms);

/**
 * @brief Update step using UWB TWR distance measurement
 *
 * Call this function each time a new TWR range measurement is available.
 * The anchor positions must be configured prior to calling this function.
 *
 * @param anchor_id Anchor index that provided the measurement (0 to SF_MAX_ANCHORS-1)
 * @param distance Measured distance to anchor (m)
 * @param timestamp_ms Current time in milliseconds
 */
void sensor_fusion_update_twr(uint8_t anchor_id, float distance, uint32_t timestamp_ms);

/**
 * @brief Update step using UWB TWR distance measurement with custom std dev
 *
 * @param anchor_id Anchor index that provided the measurement
 * @param distance Measured distance to anchor (m)
 * @param std_dev Measurement standard deviation (m)
 * @param timestamp_ms Current time in milliseconds
 */
void sensor_fusion_update_twr_with_stddev(uint8_t anchor_id, float distance,
                                          float std_dev, uint32_t timestamp_ms);

/**
 * @brief Get the current estimated state
 *
 * Retrieves the latest state estimate including position, velocity,
 * and attitude.
 *
 * @param state Pointer to state structure to fill
 */
void sensor_fusion_get_state(sensor_fusion_state_t* state);

/**
 * @brief Get just the position estimate
 *
 * Convenience function for when only position is needed.
 *
 * @param x Pointer to receive X position (m)
 * @param y Pointer to receive Y position (m)
 * @param z Pointer to receive Z position (m)
 */
void sensor_fusion_get_position(float* x, float* y, float* z);

/**
 * @brief Get just the velocity estimate
 *
 * @param vx Pointer to receive X velocity (m/s)
 * @param vy Pointer to receive Y velocity (m/s)
 * @param vz Pointer to receive Z velocity (m/s)
 */
void sensor_fusion_get_velocity(float* vx, float* vy, float* vz);

/**
 * @brief Get the attitude as Euler angles
 *
 * @param roll Pointer to receive roll angle (rad)
 * @param pitch Pointer to receive pitch angle (rad)
 * @param yaw Pointer to receive yaw angle (rad)
 */
void sensor_fusion_get_attitude(float* roll, float* pitch, float* yaw);

/**
 * @brief Get the attitude as a quaternion
 *
 * @param qw Pointer to receive quaternion W component
 * @param qx Pointer to receive quaternion X component
 * @param qy Pointer to receive quaternion Y component
 * @param qz Pointer to receive quaternion Z component
 */
void sensor_fusion_get_quaternion(float* qw, float* qx, float* qy, float* qz);

/**
 * @brief Check if the filter has valid estimates
 *
 * Returns false if the filter hasn't received enough measurements
 * or if it has diverged.
 *
 * @return true if state estimate is valid
 */
bool sensor_fusion_is_valid(void);

/**
 * @brief Set whether to use robust M-estimation for UWB updates
 *
 * Robust estimation helps reject outlier measurements but is 
 * computationally more expensive.
 *
 * @param enable True to use robust estimation, false for standard
 */
void sensor_fusion_set_robust_twr(bool enable);

/**
 * @brief ISR handler (reserved for future use)
 */
void sensor_fusion_isr(void);
