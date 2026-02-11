/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "outlier_rejection.h"
#include "sf_math.h"
#include <math.h>

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool outlier_validate_ranging(const kalmanCoreData_t* kf,
                               const distanceMeasurement_t* measurement,
                               float* innovation,
                               float* innovation_variance)
{
    /* Calculate predicted distance from current state */
    float dx = kf->S[KC_STATE_X] - measurement->x;
    float dy = kf->S[KC_STATE_Y] - measurement->y;
    float dz = kf->S[KC_STATE_Z] - measurement->z;
    float predicted_dist = arm_sqrt(dx * dx + dy * dy + dz * dz);
    
    /* Innovation: measured - predicted */
    *innovation = measurement->distance - predicted_dist;
    
    /* Calculate measurement Jacobian H = [dx/d, dy/d, dz/d, 0, 0, 0, 0, 0, 0] */
    float H[KC_STATE_DIM] = {0};
    if (predicted_dist > 1e-6f) {
        H[KC_STATE_X] = dx / predicted_dist;
        H[KC_STATE_Y] = dy / predicted_dist;
        H[KC_STATE_Z] = dz / predicted_dist;
    } else {
        /* Too close to anchor - degenerate Jacobian */
        *innovation_variance = measurement->stdDev * measurement->stdDev;
        return false;
    }
    
    /* Innovation variance: S = H * P * H^T + R */
    float HPH = 0.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            HPH += H[i] * kf->P[i][j] * H[j];
        }
    }
    float R = measurement->stdDev * measurement->stdDev;
    *innovation_variance = HPH + R;
    
    /* Reject if innovation variance is negative/zero (numerical issue) */
    if (*innovation_variance <= 0.0f) {
        return false;
    }
    
    /* Mahalanobis distance test (chi-square gating) */
    float mahal_dist = outlier_mahalanobis_distance(*innovation, *innovation_variance);
    
    /* Hard limit check */
    if (fabsf(*innovation) > INNOVATION_MAX_M) {
        return false;
    }
    
    /* Statistical test at 99% confidence */
    return (mahal_dist < MAHALANOBIS_THRESHOLD_99);
}

float outlier_mahalanobis_distance(float innovation, float innovation_variance)
{
    /* Mahalanobis distance: d = sqrt(innovation^2 / variance) */
    /* For chi-square test, return d^2 */
    if (innovation_variance <= 0.0f) {
        return INFINITY;
    }
    return (innovation * innovation) / innovation_variance;
}

bool outlier_check_state_validity(const kalmanCoreData_t* kf)
{
    /* Check for NaN or Inf in state */
    for (int i = 0; i < KC_STATE_DIM; i++) {
        if (!isfinite(kf->S[i])) {
            return false;
        }
    }
    
    /* Check for NaN or Inf in covariance (diagonal only for speed) */
    for (int i = 0; i < KC_STATE_DIM; i++) {
        if (!isfinite(kf->P[i][i]) || kf->P[i][i] < 0.0f) {
            return false;
        }
    }
    
    /* Check quaternion normalization */
    float qnorm = arm_sqrt(kf->q[0]*kf->q[0] + kf->q[1]*kf->q[1] + 
                           kf->q[2]*kf->q[2] + kf->q[3]*kf->q[3]);
    if (fabsf(qnorm - 1.0f) > 0.1f) {
        return false;  /* Quaternion not normalized */
    }
    
    return true;
}

float outlier_adaptive_measurement_noise(float measured_stddev,
                                         float innovation,
                                         float innovation_variance)
{
    /* If innovation is larger than expected, inflate measurement noise */
    /* This makes the filter trust measurements less when they're inconsistent */
    
    float normalized_innovation_sq = (innovation * innovation) / innovation_variance;
    
    /* If innovation is within 1-sigma, use nominal noise */
    if (normalized_innovation_sq < 1.0f) {
        return measured_stddev;
    }
    
    /* If innovation is 2-3 sigma, inflate by sqrt(chi-square value) */
    /* This adaptively increases R when measurements are suspect */
    float inflation_factor = arm_sqrt(normalized_innovation_sq);
    
    /* Cap at 3x inflation to avoid breaking filter */
    if (inflation_factor > 3.0f) {
        inflation_factor = 3.0f;
    }
    
    return measured_stddev * inflation_factor;
}
