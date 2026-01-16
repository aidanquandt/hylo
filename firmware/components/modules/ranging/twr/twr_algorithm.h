#pragma once

/*---------------------------------------------------------------------------
 * @file    twr_algorithm.h
 * @brief   Two-Way Ranging algorithm calculations
 * @details Implements Single-Sided TWR (SS-TWR) for UWB distance measurement.
 *          Note: Double-Sided TWR (DS-TWR) is available but not currently used.
 *---------------------------------------------------------------------------*/

#include "twr_types.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Calculate distance using Single-Sided Two-Way Ranging (SS-TWR)
 * @details SS-TWR uses 2 messages (POLL and RESPONSE) to calculate distance.
 *          Formula: distance = (round_trip_time - reply_time) * c / 2
 *
 * Message sequence:
 *   Tag:    POLL ------>
 *   Anchor:          <------ RESPONSE
 *
 * @param poll_tx_ts Tag: Timestamp when poll was transmitted
 * @param poll_rx_ts Anchor: Timestamp when poll was received
 * @param resp_tx_ts Anchor: Timestamp when response was transmitted
 * @param resp_rx_ts Tag: Timestamp when response was received
 * @param result Output: Calculated distance result
 * @return TWR_SUCCESS on success, error code otherwise
 */
twr_status_e twr_calculate_ss_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, twr_result_t* result);

/**
 * @brief Calculate distance using Double-Sided Two-Way Ranging (DS-TWR)
 * @details DS-TWR uses 3 messages (POLL, RESPONSE, FINAL) for more accurate
 *          ranging that compensates for clock drift between devices.
 *
 * Message sequence:
 *   Tag:    POLL ------>
 *   Anchor:          <------ RESPONSE
 *   Tag:    FINAL ----->
 *
 * @param poll_tx_ts Tag: Timestamp when poll was transmitted
 * @param poll_rx_ts Anchor: Timestamp when poll was received
 * @param resp_tx_ts Anchor: Timestamp when response was transmitted
 * @param resp_rx_ts Tag: Timestamp when response was received
 * @param final_tx_ts Tag: Timestamp when final was transmitted
 * @param final_rx_ts Anchor: Timestamp when final was received
 * @param result Output: Calculated distance result
 * @return TWR_SUCCESS on success, error code otherwise
 */
twr_status_e twr_calculate_ds_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts,
                                  twr_result_t* result);

/**
 * @brief Validate timestamp is reasonable
 * @param timestamp Timestamp to validate
 * @return true if valid (non-zero and within 40-bit range)
 */
bool twr_validate_timestamp(uint64_t timestamp);

/**
 * @brief Check if timestamps are in valid order
 * @param ts1 Earlier timestamp
 * @param ts2 Later timestamp
 * @param max_diff_ms Maximum expected difference in milliseconds
 * @return true if timestamps are in valid order and within expected range
 */
bool twr_validate_timestamp_order(uint64_t ts1, uint64_t ts2, uint32_t max_diff_ms);
