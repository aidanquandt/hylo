#pragma once

/*---------------------------------------------------------------------------
 * @file    ranging_manager.h
 * @brief   Ranging Manager - orchestrates ranging with multiple anchors
 * @details Manages anchor discovery, selection, and ranging schedule
 *---------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Start ranging manager operations (enables tag mode)
 * @return true if started successfully
 */
bool ranging_manager_start(void);

/**
 * @brief Stop ranging manager operations
 */
void ranging_manager_stop(void);

/**
 * @brief Check if ranging manager is active
 * @return true if actively ranging
 */
bool ranging_manager_is_active(void);

/**
 * @brief Get total number of successful ranges
 * @return Count of successful range measurements
 */
uint32_t ranging_manager_get_success_count(void);

/**
 * @brief Get total number of failed ranges
 * @return Count of failed range attempts
 */
uint32_t ranging_manager_get_failure_count(void);
