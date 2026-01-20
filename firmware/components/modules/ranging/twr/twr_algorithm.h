#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_types.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
twr_status_e twr_calculate_ss_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, twr_result_t* result);
twr_status_e twr_calculate_ds_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts,
                                  twr_result_t* result);
bool twr_validate_timestamp(uint64_t timestamp);
bool twr_validate_timestamp_order(uint64_t ts1, uint64_t ts2, uint32_t max_diff_ms);
