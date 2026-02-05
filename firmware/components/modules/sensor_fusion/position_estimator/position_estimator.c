/*---------------------------------------------------------------------------
 * @file    position_estimator.c
 * @brief   Weighted Least Squares Position Estimation
 * @details Uses Gauss-Newton iterative solver for 3D multilateration
 *---------------------------------------------------------------------------*/
#include "position_estimator.h"
#include "error_handler.h"
#include <math.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Math function wrappers (workaround for nano.specs linking issues)
 *---------------------------------------------------------------------------*/
static inline float sqrt_f(float x)
{
    float result;
    __asm__ __volatile__ ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (x));
    return result;
}

static inline float fabs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

#define sqrtf sqrt_f
#define fabsf fabs_f

/*---------------------------------------------------------------------------
 * Configuration
 *---------------------------------------------------------------------------*/
#define MAX_ITERATIONS (20U)          // Maximum solver iterations
#define CONVERGENCE_THRESHOLD (0.01f) // Convergence threshold (meters)
#define MAX_RESIDUAL_ERROR (2.0f)     // Max acceptable residual error (meters)
#define MIN_GDOP (0.1f)               // Minimum acceptable GDOP
#define MAX_GDOP (10.0f)              // Maximum acceptable GDOP

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint16_t anchor_addr;
    vec3_t anchor_position;
    float measured_distance;
    float weight; // Measurement weight (1.0 = normal, higher = more trusted)
} range_measurement_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
static range_measurement_t measurements[POSITION_ESTIMATOR_MAX_ANCHORS];
static uint8_t num_measurements = 0;
static vec3_t initial_guess       = {.x = 0.0f, .y = 0.0f, .z = 1.0f}; // Default: 1m height
static bool has_initial_guess     = false;
static position_estimator_stats_t stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
static bool solve_position_least_squares(const range_measurement_t* meas, uint8_t num_meas,
                                         vec3_t* position, float* residual_error);
static float compute_gdop(const range_measurement_t* meas, uint8_t num_meas,
                         const vec3_t* position);
static float compute_distance_3d(const vec3_t* p1, const vec3_t* p2);

/*---------------------------------------------------------------------------
 * Public Function Implementation
 *---------------------------------------------------------------------------*/

void position_estimator_init(void)
{
    num_measurements  = 0;
    has_initial_guess = false;
    memset(&stats, 0, sizeof(stats));
}

bool position_estimator_add_measurement(uint16_t anchor_addr, float distance,
                                        const vec3_t* anchor_position)
{
    if (anchor_position == NULL || distance <= 0.0f)
    {
        return false;
    }

    if (num_measurements >= POSITION_ESTIMATOR_MAX_ANCHORS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "pos_est", "Measurement buffer full");
        return false;
    }

    // Add measurement with default weight
    measurements[num_measurements].anchor_addr      = anchor_addr;
    measurements[num_measurements].anchor_position  = *anchor_position;
    measurements[num_measurements].measured_distance = distance;
    measurements[num_measurements].weight           = 1.0f; // TODO: Weight by measurement quality

    num_measurements++;
    return true;
}

bool position_estimator_compute(position_estimate_t* result)
{
    if (result == NULL)
    {
        return false;
    }

    // Initialize result
    memset(result, 0, sizeof(position_estimate_t));

    // Need at least 3 anchors for 3D position (4 for unambiguous solution)
    if (num_measurements < 3)
    {
        stats.insufficient_anchors++;
        num_measurements = 0; // Clear for next estimate
        return false;
    }

    // Solve for position using least squares
    vec3_t estimated_pos;
    float residual_error;

    if (!solve_position_least_squares(measurements, num_measurements, &estimated_pos,
                                     &residual_error))
    {
        stats.solver_failed++;
        num_measurements = 0;
        return false;
    }

    // Compute quality metric (GDOP)
    float gdop = compute_gdop(measurements, num_measurements, &estimated_pos);

    // Validate solution quality
    if (residual_error > MAX_RESIDUAL_ERROR || gdop > MAX_GDOP)
    {
        stats.high_residual_error++;
        error_handler_log(ERROR_SEVERITY_WARNING, "pos_est",
                         "Poor solution: residual=%.3f GDOP=%.2f", residual_error, gdop);
        num_measurements = 0;
        return false;
    }

    // Fill result
    result->position         = estimated_pos;
    result->residual_error   = residual_error;
    result->gdop             = gdop;
    result->num_anchors_used = num_measurements;
    result->valid            = true;

    stats.estimates_computed++;

    // Update initial guess for next iteration (helps convergence)
    initial_guess     = estimated_pos;
    has_initial_guess = true;

    // Clear measurements for next estimate
    num_measurements = 0;

    return true;
}

void position_estimator_reset(void)
{
    num_measurements = 0;
}

void position_estimator_set_initial_guess(const vec3_t* initial_position)
{
    if (initial_position != NULL)
    {
        initial_guess     = *initial_position;
        has_initial_guess = true;
    }
}

void position_estimator_get_stats(position_estimator_stats_t* stats_out)
{
    if (stats_out != NULL)
    {
        *stats_out = stats;
    }
}

void position_estimator_reset_stats(void)
{
    memset(&stats, 0, sizeof(stats));
}

/*---------------------------------------------------------------------------
 * Private Functions - Least Squares Solver
 *---------------------------------------------------------------------------*/

/**
 * @brief Gauss-Newton iterative least squares solver for multilateration
 * @details Minimizes sum of squared residuals: sum((d_measured - d_computed)^2)
 */
static bool solve_position_least_squares(const range_measurement_t* meas, uint8_t num_meas,
                                         vec3_t* position, float* residual_error)
{
    // Start from initial guess (or centroid of anchors if no guess)
    vec3_t p = has_initial_guess ? initial_guess : (vec3_t){.x = 0.0f, .y = 0.0f, .z = 1.0f};

    if (!has_initial_guess)
    {
        // Compute centroid of anchors as initial guess
        for (uint8_t i = 0; i < num_meas; i++)
        {
            p.x += meas[i].anchor_position.x;
            p.y += meas[i].anchor_position.y;
            p.z += meas[i].anchor_position.z;
        }
        p.x /= (float)num_meas;
        p.y /= (float)num_meas;
        p.z /= (float)num_meas;
    }

    // Gauss-Newton iterations
    for (uint8_t iter = 0; iter < MAX_ITERATIONS; iter++)
    {
        // Accumulate normal equations: A^T * A * delta = A^T * b
        float AtA[3][3] = {0}; // 3x3 matrix
        float Atb[3]    = {0}; // 3x1 vector

        float total_residual_sq = 0.0f;

        for (uint8_t i = 0; i < num_meas; i++)
        {
            // Compute predicted distance
            float dx             = p.x - meas[i].anchor_position.x;
            float dy             = p.y - meas[i].anchor_position.y;
            float dz             = p.z - meas[i].anchor_position.z;
            float predicted_dist = sqrtf(dx * dx + dy * dy + dz * dz);

            if (predicted_dist < 0.01f)
            {
                predicted_dist = 0.01f; // Avoid division by zero
            }

            // Residual: (measured - predicted)
            float residual = meas[i].measured_distance - predicted_dist;
            float weight   = meas[i].weight;

            // Jacobian: partial derivatives of distance w.r.t. position
            float J[3];
            J[0] = -dx / predicted_dist; // ∂d/∂x
            J[1] = -dy / predicted_dist; // ∂d/∂y
            J[2] = -dz / predicted_dist; // ∂d/∂z

            // Accumulate weighted normal equations
            for (uint8_t j = 0; j < 3; j++)
            {
                Atb[j] += weight * J[j] * residual;
                for (uint8_t k = 0; k < 3; k++)
                {
                    AtA[j][k] += weight * J[j] * J[k];
                }
            }

            total_residual_sq += weight * residual * residual;
        }

        // Solve 3x3 system: AtA * delta = Atb using Cholesky decomposition
        // For simplicity, use direct inversion (okay for 3x3)
        float det = AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) -
                    AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0]) +
                    AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);

        if (fabsf(det) < 1e-10f)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "pos_est", 
                             "Singular matrix det=%.3e", det);
            error_handler_log(ERROR_SEVERITY_WARNING, "pos_est",
                             "A1:(%.2f,%.2f,%.2f) A2:(%.2f,%.2f,%.2f)",
                             meas[0].anchor_position.x, meas[0].anchor_position.y, meas[0].anchor_position.z,
                             meas[1].anchor_position.x, meas[1].anchor_position.y, meas[1].anchor_position.z);
            error_handler_log(ERROR_SEVERITY_WARNING, "pos_est",
                             "A3:(%.2f,%.2f,%.2f) A4:(%.2f,%.2f,%.2f)",
                             meas[2].anchor_position.x, meas[2].anchor_position.y, meas[2].anchor_position.z,
                             num_meas > 3 ? meas[3].anchor_position.x : 0.0f,
                             num_meas > 3 ? meas[3].anchor_position.y : 0.0f,
                             num_meas > 3 ? meas[3].anchor_position.z : 0.0f);
            return false; // Singular matrix - poor anchor geometry
        }

        // Compute inverse and solve for delta
        float inv_det = 1.0f / det;
        float delta[3];
        delta[0] = inv_det * ((AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) * Atb[0] -
                              (AtA[0][1] * AtA[2][2] - AtA[0][2] * AtA[2][1]) * Atb[1] +
                              (AtA[0][1] * AtA[1][2] - AtA[0][2] * AtA[1][1]) * Atb[2]);
        delta[1] = inv_det * (-(AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0]) * Atb[0] +
                              (AtA[0][0] * AtA[2][2] - AtA[0][2] * AtA[2][0]) * Atb[1] -
                              (AtA[0][0] * AtA[1][2] - AtA[0][2] * AtA[1][0]) * Atb[2]);
        delta[2] = inv_det * ((AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]) * Atb[0] -
                              (AtA[0][0] * AtA[2][1] - AtA[0][1] * AtA[2][0]) * Atb[1] +
                              (AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]) * Atb[2]);

        // Update position estimate
        p.x += delta[0];
        p.y += delta[1];
        p.z += delta[2];

        // Check convergence
        float delta_norm = sqrtf(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (delta_norm < CONVERGENCE_THRESHOLD)
        {
            // Converged!
            *position       = p;
            *residual_error = sqrtf(total_residual_sq / (float)num_meas);
            return true;
        }
    }

    // Failed to converge
    error_handler_log(ERROR_SEVERITY_WARNING, "pos_est", "Solver did not converge");
    return false;
}

/**
 * @brief Compute Geometric Dilution of Precision (GDOP)
 * @details Lower GDOP = better anchor geometry, more accurate position
 *          GDOP < 2: Excellent, 2-5: Good, 5-10: Moderate, >10: Poor
 */
static float compute_gdop(const range_measurement_t* meas, uint8_t num_meas, const vec3_t* position)
{
    // GDOP = sqrt(trace((A^T * A)^-1))
    // where A is the Jacobian matrix of range equations

    float AtA[3][3] = {0};

    for (uint8_t i = 0; i < num_meas; i++)
    {
        float dx   = position->x - meas[i].anchor_position.x;
        float dy   = position->y - meas[i].anchor_position.y;
        float dz   = position->z - meas[i].anchor_position.z;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);

        if (dist < 0.01f)
            dist = 0.01f;

        float J[3] = {dx / dist, dy / dist, dz / dist};

        for (uint8_t j = 0; j < 3; j++)
        {
            for (uint8_t k = 0; k < 3; k++)
            {
                AtA[j][k] += J[j] * J[k];
            }
        }
    }

    // Compute determinant for matrix inversion
    float det = AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) -
                AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0]) +
                AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);

    if (fabsf(det) < 1e-10f)
    {
        return MAX_GDOP; // Singular - worst possible GDOP
    }

    // Compute trace of inverse matrix
    float inv_det = 1.0f / det;
    float trace   = inv_det * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1] +
                             AtA[0][0] * AtA[2][2] - AtA[0][2] * AtA[2][0] +
                             AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]);

    return sqrtf(fabsf(trace));
}

static float compute_distance_3d(const vec3_t* p1, const vec3_t* p2)
{
    float dx = p1->x - p2->x;
    float dy = p1->y - p2->y;
    float dz = p1->z - p2->z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}
