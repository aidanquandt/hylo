/*---------------------------------------------------------------------------
 * @file    imu_test.h
 * @brief   IMU hardware connection test module
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "imu_port.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Get current accelerometer readings
 * @param accel Pointer to structure to store accelerometer data
 * @return true if data is valid (IMU is active), false otherwise
 */
bool imu_test_get_accel(imu_sensor_data_t *accel);

/**
 * @brief Get current gyroscope readings
 * @param gyro Pointer to structure to store gyroscope data
 * @return true if data is valid (IMU is active), false otherwise
 */
bool imu_test_get_gyro(imu_sensor_data_t *gyro);

/**
 * @brief Get current temperature reading
 * @return Temperature in degrees Celsius
 */
float imu_test_get_temperature(void);

/**
 * @brief Check if IMU is ready and active
 * @return true if IMU is initialized and ready, false otherwise
 */
bool imu_test_is_ready(void);
