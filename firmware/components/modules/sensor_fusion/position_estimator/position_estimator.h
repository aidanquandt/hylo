#pragma once

/*---------------------------------------------------------------------------
 * @file    position_estimator.h
 * @brief   3D Position Estimation via Weighted Least Squares Trilateration
 * @details Estimates position from multiple range measurements to anchors
 *          with known positions using iterative least squares solver
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Configuration
 *---------------------------------------------------------------------------*/
#define POSITION_ESTIMATOR_MAX_ANCHORS (8U) // Maximum anchors for one position estimate

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    vec3_t position;           // Estimated 3D position (meters)
    float residual_error;      // RMS residual error (meters)
    float gdop;                // Geometric Dilution of Precision
    uint8_t num_anchors_used;  // Number of anchors used in estimate
    bool valid;                // True if estimate is valid
} position_estimate_t;

typedef struct
{
    uint32_t estimates_computed;    // Total successful position estimates
    uint32_t insufficient_anchors;  // Rejected: not enough anchors (need 3+)
    uint32_t solver_failed;         // Solver failed to converge
    uint32_t high_residual_error;   // High residual error (poor geometry/noise)
} position_estimator_stats_t;

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize position estimator
 */
void position_estimator_init(void);

/**
 * @brief Add a range measurement for the next position estimate
 * @param anchor_addr Anchor address (for logging/debugging)
 * @param distance Measured distance to anchor (meters)
 * @param anchor_position Known anchor position (meters)
 * @return true if measurement added successfully
 * @note Call this multiple times (one per anchor), then call compute()
 */
bool position_estimator_add_measurement(uint16_t anchor_addr, float distance,
                                        const vec3_t* anchor_position);

/**
 * @brief Compute position estimate from accumulated measurements
 * @param result Output position estimate with quality metrics
 * @return true if position computed successfully
 * @note This clears accumulated measurements after computation
 */
bool position_estimator_compute(position_estimate_t* result);

/**
 * @brief Reset/clear all accumulated measurements
 */
void position_estimator_reset(void);

/**
 * @brief Set initial position guess for solver (improves convergence)
 * @param initial_position Initial position guess (meters)
 */
void position_estimator_set_initial_guess(const vec3_t* initial_position);

/**
 * @brief Get estimator statistics
 */
void position_estimator_get_stats(position_estimator_stats_t* stats);

/**
 * @brief Reset statistics counters
 */
void position_estimator_reset_stats(void);
