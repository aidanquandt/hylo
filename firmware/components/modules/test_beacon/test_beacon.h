/*---------------------------------------------------------------------------
 * @file    test_beacon.h
 * @brief   Test beacon module - listens for UWB messages and responds
 *
 * @note    This is demo/test code that uses the uwb.c API. It listens for
 *          incoming UWB messages and responds with an auto-incrementing counter.
 *          This demonstrates how application modules should use the UWB
 *          infrastructure. Can be excluded from production builds.
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
 * @brief Enable or disable the test beacon responder (deprecated - use beacon.set mode)
 * @param enable true for responder mode, false for master mode
 * @note Use UART command "beacon.set mode responder/master" instead
 * @deprecated Use mode setting via UART commands
 */
void test_beacon_enable(bool enable);

/**
 * @brief Check if test beacon is in responder mode
 * @return true if in responder mode, false if in master mode
 */
bool test_beacon_is_enabled(void);

/**
 * @brief Set the beacon counter value
 * @param value Counter value to use for next response (0-65535)
 * @note Counter increments automatically after each response sent
 */
void test_beacon_set_value(uint16_t value);

/**
 * @brief Enable or disable auto-increment mode (deprecated)
 * @param enable Ignored - counter always auto-increments
 * @note This function is deprecated. Counter always increments on each response.
 */
void test_beacon_set_auto_increment(bool enable);

/**
 * @brief Get current beacon statistics
 * @param counter Output: current counter value (next response number)
 * @param attempts Output: number of responses sent
 * @param auto_inc Output: always true (counter always auto-increments)
 */
void test_beacon_get_stats(uint16_t* counter, uint32_t* attempts, bool* auto_inc);
