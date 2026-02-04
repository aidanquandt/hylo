/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_algorithm.h"
#include "uwb_port.h"
#include "common.h"

/*---------------------------------------------------------------------------
 * Private Definitions
 *---------------------------------------------------------------------------*/
#define TWR_MAX_REASONABLE_DISTANCE_M 1000.0f
#define TWR_MAX_TIMESTAMP_DIFF_MS 100U
#define TWR_MIN_TIMESTAMP_DIFF_DTU 1000ULL

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

twr_status_e twr_calculate_ss_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, twr_result_t* result)
{
    if (result == NULL)
    {
        return TWR_ERROR_NULL_PTR;
    }

    memset(result, 0, sizeof(twr_result_t));
    result->valid = false;

    if (!twr_validate_timestamp(poll_tx_ts) || !twr_validate_timestamp(poll_rx_ts) ||
        !twr_validate_timestamp(resp_tx_ts) || !twr_validate_timestamp(resp_rx_ts))
    {
        return TWR_ERROR_INVALID_TIMESTAMP;
    }

    int64_t round_trip = twr_timestamp_diff(poll_tx_ts, resp_rx_ts);
    int64_t reply_time = twr_timestamp_diff(poll_rx_ts, resp_tx_ts);

    if (round_trip <= 0 || reply_time <= 0)
    {
        return TWR_ERROR_INVALID_TIMESTAMP;
    }

    if (round_trip < reply_time)
    {
        // Reply time can't be longer than round trip
        return TWR_ERROR_CALCULATION_FAILED;
    }

    // Calculate time of flight: ToF = (round_trip - reply_time) / 2
    int64_t tof_dtu = (round_trip - reply_time) / 2;

    if (tof_dtu < (int64_t)TWR_MIN_TIMESTAMP_DIFF_DTU)
    {
        // Too small, likely measurement error
        return TWR_ERROR_CALCULATION_FAILED;
    }

    float distance = twr_dtu_to_meters(tof_dtu);
    if (distance < 0.0f || distance > TWR_MAX_REASONABLE_DISTANCE_M)
    {
        return TWR_ERROR_OUT_OF_RANGE;
    }

    // Success!
    result->distance_m = distance;
    result->valid      = true;

    return TWR_SUCCESS;
}

twr_status_e twr_calculate_ds_twr(uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t resp_tx_ts,
                                  uint64_t resp_rx_ts, uint64_t final_tx_ts, uint64_t final_rx_ts,
                                  twr_result_t* result)
{
    if (result == NULL)
    {
        return TWR_ERROR_NULL_PTR;
    }

    // Initialize result
    memset(result, 0, sizeof(twr_result_t));
    result->valid = false;

    // Validate all timestamps
    if (!twr_validate_timestamp(poll_tx_ts) || !twr_validate_timestamp(poll_rx_ts) ||
        !twr_validate_timestamp(resp_tx_ts) || !twr_validate_timestamp(resp_rx_ts) ||
        !twr_validate_timestamp(final_tx_ts) || !twr_validate_timestamp(final_rx_ts))
    {
        return TWR_ERROR_INVALID_TIMESTAMP;
    }

    // Double-sided TWR formula (compensates for clock drift):
    // ToF = (Da × Db - Ra × Rb) / (Ra + Rb + Da + Db)
    //
    // Where:
    //   Ra = reply time at responder (resp_tx - poll_rx)
    //   Rb = reply time at initiator (final_tx - resp_rx)
    //   Da = round trip at responder (final_rx - resp_tx) = Rb + 2×ToF
    //   Db = round trip at initiator (resp_rx - poll_tx) = Ra + 2×ToF

    int64_t Ra = twr_timestamp_diff(poll_rx_ts, resp_tx_ts);
    int64_t Rb = twr_timestamp_diff(resp_rx_ts, final_tx_ts);
    int64_t Da = twr_timestamp_diff(resp_tx_ts, final_rx_ts);
    int64_t Db = twr_timestamp_diff(poll_tx_ts, resp_rx_ts);

    // Sanity checks
    if (Ra <= 0 || Rb <= 0 || Da <= 0 || Db <= 0)
    {
        return TWR_ERROR_INVALID_TIMESTAMP;
    }

    // Calculate time of flight
    // Using double precision for intermediate calculations
    double Ra_d = (double)Ra;
    double Rb_d = (double)Rb;
    double Da_d = (double)Da;
    double Db_d = (double)Db;

    // DS-TWR formula: ToF = (Da × Db - Ra × Rb) / (Ra + Rb + Da + Db)
    // Where: Da = Rb + 2×ToF, Db = Ra + 2×ToF
    // This gives: Da×Db - Ra×Rb = 2×ToF×(Ra + Rb + 2×ToF)
    double numerator   = Da_d * Db_d - Ra_d * Rb_d;
    double denominator = Ra_d + Rb_d + Da_d + Db_d;

    if (denominator < (double)TWR_MIN_TIMESTAMP_DIFF_DTU)
    {
        return TWR_ERROR_CALCULATION_FAILED;
    }

    double tof_dtu = numerator / denominator;

    if (tof_dtu < 0)
    {
        // Negative ToF indicates calculation error
        return TWR_ERROR_CALCULATION_FAILED;
    }

    float distance = twr_dtu_to_meters((int64_t)tof_dtu);
    if (distance < 0.0f || distance > TWR_MAX_REASONABLE_DISTANCE_M)
    {
        return TWR_ERROR_OUT_OF_RANGE;
    }

    result->distance_m = distance;
    result->valid      = true;

    return TWR_SUCCESS;
}

bool twr_validate_timestamp(uint64_t timestamp)
{
    uint64_t mask_40bit = 0xFFFFFFFFFFULL;
    return (timestamp != 0) && (timestamp == (timestamp & mask_40bit));
}

bool twr_validate_timestamp_order(uint64_t ts1, uint64_t ts2, uint32_t max_diff_ms)
{
    if (!twr_validate_timestamp(ts1) || !twr_validate_timestamp(ts2))
    {
        return false;
    }

    int64_t diff_dtu = twr_timestamp_diff(ts1, ts2);

    if (diff_dtu <= 0)
    {
        return false;
    }

    int64_t max_diff_dtu = (int64_t)(UWB_MS_TO_DTUH(max_diff_ms));

    return (diff_dtu <= max_diff_dtu);
}
