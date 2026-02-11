#pragma once

#include <stdbool.h>
#include "kalman_core.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

/** Chi-square threshold for 1-DOF measurement at 95% confidence */
#define MAHALANOBIS_THRESHOLD_95 (3.841f)

/** Chi-square threshold for 1-DOF measurement at 99% confidence */
#define MAHALANOBIS_THRESHOLD_99 (6.635f)

/** Maximum allowed innovation (meters) - hard limit for safety */
#define INNOVATION_MAX_M (2.0f)

/*---------------------------------------------------------------------------
 * Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Validate UWB ranging measurement using innovation statistics
 * @param kf Kalman filter state
 * @param measurement Distance measurement to validate
 * @param[out] innovation Calculated innovation (predicted - measured)
 * @param[out] innovation_variance Uncertainty in innovation
 * @return true if measurement passes validation, false if outlier
 */
bool outlier_validate_ranging(const kalmanCoreData_t* kf,
                               const distanceMeasurement_t* measurement,
                               float* innovation,
                               float* innovation_variance);

/**
 * @brief Calculate Mahalanobis distance for measurement
 * @param innovation Measurement residual
 * @param innovation_variance Residual covariance
 * @return Mahalanobis distance (chi-square distributed)
 */
float outlier_mahalanobis_distance(float innovation, float innovation_variance);

/**
 * @brief Check if position estimate is diverging (NaN, infinite, etc)
 * @param kf Kalman filter state
 * @return true if state is valid, false if diverged
 */
bool outlier_check_state_validity(const kalmanCoreData_t* kf);

/**
 * @brief Adaptive measurement noise based on innovation consistency
 * @param measured_stddev Sensor-reported measurement noise
 * @param innovation Recent innovation magnitude
 * @param innovation_variance Expected innovation variance
 * @return Adjusted measurement noise
 */
float outlier_adaptive_measurement_noise(float measured_stddev,
                                         float innovation,
                                         float innovation_variance);
