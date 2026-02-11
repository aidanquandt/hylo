#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "imu.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

/** Number of samples for static calibration */
#define CALIB_SAMPLES 200

/** Maximum acceleration for stationary detection (m/s²) */
#define CALIB_ACCEL_THRESHOLD_MS2 (0.5f)

/** Expected gravity magnitude (m/s²) */
#define CALIB_GRAVITY_MS2 (9.81f)

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/

typedef struct {
    vec3_t accel_bias;       // Accelerometer bias (m/s²)
    vec3_t gyro_bias;        // Gyroscope bias (rad/s)
    float accel_scale[3];    // Accelerometer scale factors
    float gyro_scale[3];     // Gyroscope scale factors
    float temperature_ref;   // Reference temperature for bias (°C)
    bool is_calibrated;      // Calibration valid flag
    uint32_t calib_timestamp_ms; // When calibration was performed
} imu_calibration_t;

/*---------------------------------------------------------------------------
 * Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Perform static IMU calibration (device must be stationary)
 * @param calib Output calibration parameters
 * @param timeout_ms Maximum time to wait for stable readings
 * @return true if calibration successful, false otherwise
 * 
 * @note Device must remain stationary during this procedure
 * @note Should be called during startup or on user command
 */
bool imu_calibrate_static(imu_calibration_t* calib, uint32_t timeout_ms);

/**
 * @brief Apply calibration to raw IMU reading
 * @param calib Calibration parameters
 * @param raw_accel Raw accelerometer reading (m/s²)
 * @param raw_gyro Raw gyroscope reading (rad/s)
 * @param[out] corrected_accel Bias-corrected accelerometer
 * @param[out] corrected_gyro Bias-corrected gyroscope
 */
void imu_apply_calibration(const imu_calibration_t* calib,
                           const vec3_t* raw_accel,
                           const vec3_t* raw_gyro,
                           vec3_t* corrected_accel,
                           vec3_t* corrected_gyro);

/**
 * @brief Temperature compensation for IMU bias drift
 * @param calib Base calibration at reference temperature
 * @param current_temp Current IMU temperature (°C)
 * @param[in,out] accel Accelerometer reading to compensate
 * @param[in,out] gyro Gyroscope reading to compensate
 * 
 * @note Requires thermal characterization of your specific IMU
 */
void imu_compensate_temperature(const imu_calibration_t* calib,
                                 float current_temp,
                                 vec3_t* accel,
                                 vec3_t* gyro);

/**
 * @brief Check if device is stationary (for ZUPT or recalibration)
 * @param accel Current accelerometer reading
 * @param gyro Current gyroscope reading
 * @param samples Number of consecutive samples below threshold
 * @return true if stationary, false if moving
 */
bool imu_is_stationary(const vec3_t* accel, const vec3_t* gyro, uint32_t samples);

/**
 * @brief Save calibration to non-volatile memory
 * @param calib Calibration to save
 * @return true if successful
 */
bool imu_save_calibration(const imu_calibration_t* calib);

/**
 * @brief Load calibration from non-volatile memory
 * @param calib Output calibration
 * @return true if valid calibration loaded
 */
bool imu_load_calibration(imu_calibration_t* calib);
