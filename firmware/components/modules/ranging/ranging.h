#pragma once

/*---------------------------------------------------------------------------
 * @file    ranging.h
 * @brief   SS-TWR Ranging module - coordinates Tag and Anchor modes
 * @details Top-level ranging module using Single-Sided Two-Way Ranging (2-message protocol).
 *          Allows runtime selection between tag (initiator) and anchor (responder) modes.
 *---------------------------------------------------------------------------*/

#include "common.h"
#include "twr/twr_types.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Ranging mode selection
 */
typedef enum
{
    RANGING_MODE_DISABLED = 0, // Ranging disabled
    RANGING_MODE_TAG,          // Tag/Initiator mode
    RANGING_MODE_ANCHOR        // Anchor/Responder mode
} ranging_mode_e;

/**
 * @brief Ranging module status
 */
typedef struct
{
    ranging_mode_e mode;         // Current mode
    bool active;                 // Module is active
    uint32_t successful_ranges;  // Total successful ranges
    uint32_t failed_ranges;      // Total failed ranges
    uint32_t messages_processed; // Total TWR messages processed
} ranging_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Set ranging mode
 * @param mode Desired ranging mode
 * @return true if mode set successfully
 */
bool ranging_set_mode(ranging_mode_e mode);

/**
 * @brief Get current ranging mode
 * @return Current ranging mode
 */
ranging_mode_e ranging_get_mode(void);

/**
 * @brief Get ranging module status
 * @param status Output: Status information
 */
void ranging_get_status(ranging_status_t* status);

/*---------------------------------------------------------------------------
 * Tag-Specific Functions (only work in TAG mode)
 *---------------------------------------------------------------------------*/

/**
 * @brief Initiate 2-message ranging to specific anchor
 * @details Tag sends POLL, waits for RESPONSE with timestamps
 * @param anchor_addr Address of anchor to range to
 * @return true if ranging started successfully
 * @note Only works when mode = RANGING_MODE_TAG
 */
bool ranging_tag_start(uint16_t anchor_addr);

/**
 * @brief Check if tag is currently ranging
 * @return true if ranging in progress
 * @note Only works when mode = RANGING_MODE_TAG
 */
bool ranging_tag_is_active(void);

/**
 * @brief Get last ranging result
 * @param distance_m Output: Distance in meters
 * @param rssi_dbm Output: RSSI in dBm (optional, can be NULL)
 * @return true if valid result available
 * @note Only works when mode = RANGING_MODE_TAG
 */
bool ranging_tag_get_result(float* distance_m, float* rssi_dbm);

/**
 * @brief Cancel current ranging operation
 * @note Only works when mode = RANGING_MODE_TAG
 */
void ranging_tag_cancel(void);

/*---------------------------------------------------------------------------
 * Anchor-Specific Functions (only work in ANCHOR mode)
 *---------------------------------------------------------------------------*/

/**
 * @brief Set anchor address
 * @param address Our anchor address
 * @note Can be called even when not in ANCHOR mode (preconfiguration)
 */
void ranging_anchor_set_address(uint16_t address);

/**
 * @brief Get anchor address
 * @return Current anchor address
 */
uint16_t ranging_anchor_get_address(void);

/**
 * @brief Get number of ranging responses sent
 * @return Total responses sent
 * @note Only meaningful when mode = RANGING_MODE_ANCHOR
 */
uint32_t ranging_anchor_get_response_count(void);
