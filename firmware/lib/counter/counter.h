/*---------------------------------------------------------------------------
 * @file    counter.h
 * @brief   Utility functions for counter operations
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Increment uint8_t counter and check if threshold reached
 * @param counter Pointer to uint8_t counter variable
 * @param threshold Threshold value to check against
 * @return true if counter reached threshold (counter is reset to 0), false otherwise
 */
bool counter_uint8_t(uint8_t *counter, uint8_t threshold);

/**
 * @brief Increment uint16_t counter and check if threshold reached
 * @param counter Pointer to uint16_t counter variable
 * @param threshold Threshold value to check against
 * @return true if counter reached threshold (counter is reset to 0), false otherwise
 */
bool counter_uint16_t(uint16_t *counter, uint16_t threshold);

/**
 * @brief Increment uint32_t counter and check if threshold reached
 * @param counter Pointer to uint32_t counter variable
 * @param threshold Threshold value to check against
 * @return true if counter reached threshold (counter is reset to 0), false otherwise
 */
bool counter_uint32_t(uint32_t *counter, uint32_t threshold);
