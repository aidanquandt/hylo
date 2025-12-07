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
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Perform a soft reset on the IMU device
 *
 * Performs a software reset of the IMU chip. This should only be called
 * if the device has been initialized and is ready.
 *
 * @return true if reset was successful, false if device is not ready or reset failed
 */
bool imu_soft_reset(void);