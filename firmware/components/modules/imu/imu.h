/*---------------------------------------------------------------------------
 * @file    imu.h
 * @brief   IMU hardware connection test module
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief IMU state
 */
typedef enum
{
    IMU_STATE_STARTUP,        ///< Starting up
    IMU_STATE_INITIALIZATION, ///< Initializing hardware
    IMU_STATE_ACTIVE,         ///< Active and reading data
    IMU_STATE_FAULTED         ///< Error state
} imu_state_e;

/**
 * @brief 3D vector data
 */
typedef struct
{
    float x;
    float y;
    float z;
} imu_vector3_t;

/**
 * @brief IMU status information
 */
typedef struct
{
    imu_state_e state;   ///< Current state
    uint8_t chip_id;     ///< Chip ID read from device
    uint32_t fault_code; ///< Fault code (0 = no fault)
} imu_status_t;

/**
 * @brief IMU measurement data
 */
typedef struct
{
    imu_vector3_t accel; ///< Acceleration (m/s^2)
    imu_vector3_t gyro;  ///< Gyroscope (deg/s)
    float temperature;   ///< Temperature (degrees C)
} imu_data_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Get current IMU status
 * @param status Output: status information
 */
void imu_get_status(imu_status_t* status);

/**
 * @brief Get all IMU measurements
 * @param data Output: measurement data
 * @return true if successful, false if IMU not active
 */
bool imu_get_data(imu_data_t* data);

/**
 * @brief Get accelerometer data only
 * @param accel Output: acceleration vector (m/s^2)
 * @return true if successful, false if IMU not active
 */
bool imu_get_accel(imu_vector3_t* accel);

/**
 * @brief Get gyroscope data only
 * @param gyro Output: gyroscope vector (deg/s)
 * @return true if successful, false if IMU not active
 */
bool imu_get_gyro(imu_vector3_t* gyro);

/**
 * @brief Get temperature only
 * @param temp Output: temperature (degrees C)
 * @return true if successful, false if IMU not active
 */
bool imu_get_temp(float* temp);

/**
 * @brief Perform a soft reset on the IMU device
 * @return true if reset was successful, false if device is not ready or reset failed
 */
bool imu_soft_reset(void);