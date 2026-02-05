# Position Estimator

## Overview

Weighted Least Squares 3D position estimator for UWB multilateration. Computes position from multiple range measurements to anchors with known positions.

## Algorithm

- **Method**: Gauss-Newton iterative least squares
- **Input**: Distance measurements to 3+ anchors with known positions
- **Output**: 3D position estimate with quality metrics (residual error, GDOP)
- **Solver**: Direct 3x3 matrix inversion (robust for 3-8 anchors)

## Usage

### Basic Flow

```c
// 1. Initialize once at startup
position_estimator_init();

// 2. For each position estimate, add measurements from all visible anchors
position_estimator_add_measurement(anchor_addr_1, distance_1, &anchor_pos_1);
position_estimator_add_measurement(anchor_addr_2, distance_2, &anchor_pos_2);
position_estimator_add_measurement(anchor_addr_3, distance_3, &anchor_pos_3);
position_estimator_add_measurement(anchor_addr_4, distance_4, &anchor_pos_4);

// 3. Compute position estimate
position_estimate_t result;
if (position_estimator_compute(&result)) {
    // Success! Use result.position (x, y, z)
    printf("Position: (%.2f, %.2f, %.2f)\n", 
           result.position.x, result.position.y, result.position.z);
    printf("Quality: GDOP=%.2f, residual=%.3fm\n", 
           result.gdop, result.residual_error);
}

// Note: compute() automatically clears measurements for next estimate
```

### Quality Metrics

**GDOP (Geometric Dilution of Precision):**
- < 2: Excellent geometry
- 2-5: Good geometry
- 5-10: Moderate geometry
- \> 10: Poor geometry (estimate rejected)

**Residual Error:**
- RMS error between measured and computed distances
- < 0.5m: Excellent
- 0.5-1.0m: Good
- \> 2.0m: Poor (estimate rejected)

## Requirements

- **Minimum anchors**: 3 (for 3D position)
- **Recommended**: 4+ anchors (better accuracy, unambiguous solution)
- **Known anchor positions**: Must be accurate (errors directly affect position estimate)

## Integration with sensor_fusion

The sensor fusion module automatically uses this estimator:

1. Ranging events arrive via `sensor_fusion_push_event()`
2. `process_ranging_event()` calls `position_estimator_add_measurement()`
3. After MIN_ANCHORS_FOR_ESTIMATE measurements, `position_estimator_compute()` is called
4. Result is stored in `position_estimate` structure
5. Retrieved via `sensor_fusion_get_position()`

## Performance

- **Convergence**: Typically 3-10 iterations
- **Max iterations**: 20 (fails if not converged)
- **Memory**: ~1KB static allocation
- **CPU**: ~2-5ms per estimate on Cortex-M7 @ 480MHz

## Troubleshooting

**Solver fails to converge:**
- Poor anchor geometry (all on one plane or line)
- Very large distance errors
- Anchors too close to each other

**High residual error:**
- Measurement noise/multipath
- Incorrect anchor positions
- Time sync errors (for TDOA)

**High GDOP:**
- Anchors poorly distributed (all on one side)
- Anchors coplanar with tag
- Best: Anchors forming a tetrahedron around tag
