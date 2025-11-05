/*---------------------------------------------------------------------------
 * @file    dw3000_test.h
 * @brief   DW3000 hardware connection test module
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
 * @brief Test DW3000 SPI communication and device ID
 * @return true if device responds correctly, false otherwise
 */
bool dw3000_test_device_id(void);

/**
 * @brief Get the last measured device ID
 * @return 32-bit device ID value, or 0 if not yet read
 */
uint32_t dw3000_test_get_device_id(void);

/**
 * @brief Get the last measured temperature
 * @return Temperature in degrees Celsius, or 0.0f if not yet read
 */
float dw3000_test_get_temperature(void);

/**
 * @brief Get the last measured voltage
 * @return Voltage in volts, or 0.0f if not yet read
 */
float dw3000_test_get_voltage(void);

/**
 * @brief Check if hardware is ready
 * @return true if DW3000 is initialized and ready
 */
bool dw3000_test_is_ready(void);
