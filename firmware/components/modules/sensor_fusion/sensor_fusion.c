/*---------------------------------------------------------------------------
 * @file    sensor_fusion.c
 * @brief   Error-State Kalman Filter (ESKF) for IMU + UWB TWR fusion
 * @details Implements ESKF to fuse IMU predictions with UWB TWR corrections
 *          from 4 anchors. Uses quaternion for attitude representation.
 *
 * State vector (nominal): [x, y, z, vx, vy, vz] (6D)
 * Error state: [δp, δv, δθ] (9D) - position, velocity, attitude error
 * Attitude tracked separately as quaternion
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "module.h"
#include <math.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define GRAVITY_MAG 9.81f
#define DEG_TO_RAD (3.14159265359f / 180.0f)

// Process noise tuning parameters
#define W_ACCEL 2.0f      // Accelerometer noise std [m/s^2]
#define W_GYRO 0.1f       // Gyroscope noise std [rad/s]

// Measurement noise
#define STD_UWB_RANGE 0.1f  // UWB range measurement std [m]

// Outlier rejection threshold (Mahalanobis distance)
#define OUTLIER_THRESHOLD 5.0f

// Minimum covariance to prevent numerical issues
#define MIN_COVARIANCE 1e-6f

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/

/**
 * @brief ESKF internal state
 */
typedef struct
{
    // Nominal state
    float pos[3];       // Position [x, y, z]
    float vel[3];       // Velocity [vx, vy, vz]
    eskf_quat_t quat;   // Attitude quaternion (body to world)

    // Error state covariance P (9x9)
    float P[ESKF_ERROR_DIM][ESKF_ERROR_DIM];

    // Anchor configuration
    eskf_anchor_config_t anchors;

    // Statistics
    eskf_stats_t stats;

    // Initialization flag
    bool initialized;
} eskf_state_internal_t;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void sensor_fusion_init(void);

extern const module_S sensor_fusion_module;

const module_S sensor_fusion_module = {
    .module_name = "sensor_fusion",
    .module_init = sensor_fusion_init,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC eskf_state_internal_t eskf = {0};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void quat_normalize(eskf_quat_t* q);
STATIC void quat_multiply(const eskf_quat_t* q1, const eskf_quat_t* q2, eskf_quat_t* result);
STATIC void quat_to_rotation_matrix(const eskf_quat_t* q, float R[3][3]);
STATIC void rotation_vector_to_quat(const float rv[3], eskf_quat_t* q);
STATIC void mat3_vec3_mult(const float M[3][3], const float v[3], float result[3]);
STATIC void skew_symmetric(const float v[3], float S[3][3]);
STATIC float vec3_norm(const float v[3]);
STATIC void mat_transpose_3x3(const float A[3][3], float At[3][3]);
STATIC void rodrigues_exp(const float w[3], float R[3][3]);

/*---------------------------------------------------------------------------
 * Private Function Implementations - Quaternion/Matrix Operations
 *---------------------------------------------------------------------------*/

/**
 * @brief Normalize quaternion to unit length
 */
STATIC void quat_normalize(eskf_quat_t* q)
{
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (norm > 1e-10f)
    {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

/**
 * @brief Quaternion multiplication: result = q1 * q2
 */
STATIC void quat_multiply(const eskf_quat_t* q1, const eskf_quat_t* q2, eskf_quat_t* result)
{
    result->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

/**
 * @brief Convert quaternion to 3x3 rotation matrix (body to world)
 */
STATIC void quat_to_rotation_matrix(const eskf_quat_t* q, float R[3][3])
{
    float w = q->w, x = q->x, y = q->y, z = q->z;

    R[0][0] = 1.0f - 2.0f * (y * y + z * z);
    R[0][1] = 2.0f * (x * y - w * z);
    R[0][2] = 2.0f * (x * z + w * y);

    R[1][0] = 2.0f * (x * y + w * z);
    R[1][1] = 1.0f - 2.0f * (x * x + z * z);
    R[1][2] = 2.0f * (y * z - w * x);

    R[2][0] = 2.0f * (x * z - w * y);
    R[2][1] = 2.0f * (y * z + w * x);
    R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

/**
 * @brief Convert rotation vector (angle-axis) to quaternion
 *        Using: q = [cos(θ/2), sin(θ/2) * axis]
 */
STATIC void rotation_vector_to_quat(const float rv[3], eskf_quat_t* q)
{
    float angle = vec3_norm(rv);
    if (angle < 1e-10f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
    else
    {
        float half_angle = 0.5f * angle;
        float s = sinf(half_angle) / angle;
        q->w = cosf(half_angle);
        q->x = rv[0] * s;
        q->y = rv[1] * s;
        q->z = rv[2] * s;
    }
}

/**
 * @brief Matrix-vector multiplication: result = M * v
 */
STATIC void mat3_vec3_mult(const float M[3][3], const float v[3], float result[3])
{
    for (int i = 0; i < 3; i++)
    {
        result[i] = M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2];
    }
}

/**
 * @brief Create skew-symmetric matrix from vector
 *        [v]_x = [ 0   -vz   vy ]
 *                [ vz   0   -vx ]
 *                [-vy   vx   0  ]
 */
STATIC void skew_symmetric(const float v[3], float S[3][3])
{
    S[0][0] = 0.0f;
    S[0][1] = -v[2];
    S[0][2] = v[1];
    S[1][0] = v[2];
    S[1][1] = 0.0f;
    S[1][2] = -v[0];
    S[2][0] = -v[1];
    S[2][1] = v[0];
    S[2][2] = 0.0f;
}

/**
 * @brief Compute vector 3D norm
 */
STATIC float vec3_norm(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * @brief Transpose 3x3 matrix
 */
STATIC void mat_transpose_3x3(const float A[3][3], float At[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            At[i][j] = A[j][i];
        }
    }
}

/**
 * @brief Rodrigues formula: compute rotation matrix from rotation vector
 *        R = I + sin(θ)[ω]_x + (1-cos(θ))[ω]_x^2
 */
STATIC void rodrigues_exp(const float w[3], float R[3][3])
{
    float angle = vec3_norm(w);
    if (angle < 1e-10f)
    {
        // Identity matrix
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                R[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return;
    }

    float axis[3] = {w[0] / angle, w[1] / angle, w[2] / angle};
    float s = sinf(angle);
    float c = cosf(angle);
    float t = 1.0f - c;

    R[0][0] = t * axis[0] * axis[0] + c;
    R[0][1] = t * axis[0] * axis[1] - s * axis[2];
    R[0][2] = t * axis[0] * axis[2] + s * axis[1];

    R[1][0] = t * axis[0] * axis[1] + s * axis[2];
    R[1][1] = t * axis[1] * axis[1] + c;
    R[1][2] = t * axis[1] * axis[2] - s * axis[0];

    R[2][0] = t * axis[0] * axis[2] - s * axis[1];
    R[2][1] = t * axis[1] * axis[2] + s * axis[0];
    R[2][2] = t * axis[2] * axis[2] + c;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - ESKF Core
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize filter with default values
 */
STATIC void sensor_fusion_init(void)
{
    memset(&eskf, 0, sizeof(eskf));

    // Initialize quaternion to identity (no rotation)
    eskf.quat.w = 1.0f;
    eskf.quat.x = 0.0f;
    eskf.quat.y = 0.0f;
    eskf.quat.z = 0.0f;

    // Initialize covariance with reasonable uncertainty
    // Position: 1m std, Velocity: 0.1 m/s std, Attitude: 0.1 rad std
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            eskf.P[i][j] = 0.0f;
        }
    }

    // Position covariance (0-2)
    eskf.P[0][0] = 1.0f;
    eskf.P[1][1] = 1.0f;
    eskf.P[2][2] = 1.0f;

    // Velocity covariance (3-5)
    eskf.P[3][3] = 0.01f;
    eskf.P[4][4] = 0.01f;
    eskf.P[5][5] = 0.01f;

    // Attitude covariance (6-8)
    eskf.P[6][6] = 0.01f;
    eskf.P[7][7] = 0.01f;
    eskf.P[8][8] = 0.01f;

    eskf.initialized = false;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void sensor_fusion_set_anchors(const eskf_vec3_t anchors[ESKF_NUM_ANCHORS])
{
    for (uint8_t i = 0; i < ESKF_NUM_ANCHORS; i++)
    {
        eskf.anchors.position[i] = anchors[i];
    }
    eskf.anchors.configured = true;
}

void sensor_fusion_set_initial_position(const eskf_vec3_t* position)
{
    eskf.pos[0] = position->x;
    eskf.pos[1] = position->y;
    eskf.pos[2] = position->z;

    eskf.vel[0] = 0.0f;
    eskf.vel[1] = 0.0f;
    eskf.vel[2] = 0.0f;

    eskf.initialized = true;
}

/**
 * @brief IMU prediction step
 * 
 * Nominal state propagation:
 *   p(k) = p(k-1) + v(k-1)*dt + 0.5*R*(a - g)*dt^2
 *   v(k) = v(k-1) + R*(a - g)*dt
 *   q(k) = q(k-1) * Δq(ω*dt)
 *
 * Error state covariance propagation:
 *   P(k) = Fx * P(k-1) * Fx^T + Fi * Qi * Fi^T
 */
void sensor_fusion_imu_update(const eskf_imu_meas_t* imu, float dt)
{
    if (!eskf.initialized || !eskf.anchors.configured)
    {
        return;
    }

    float dt2 = dt * dt;

    // Get rotation matrix from current quaternion
    float R[3][3];
    quat_to_rotation_matrix(&eskf.quat, R);

    // Accelerometer in body frame (already in m/s^2)
    float accel_body[3] = {imu->accel.x, imu->accel.y, imu->accel.z};

    // Rotate acceleration to world frame
    float accel_world[3];
    mat3_vec3_mult(R, accel_body, accel_world);

    // Subtract gravity (gravity is in -Z direction in world frame)
    accel_world[2] -= GRAVITY_MAG;

    // Gyroscope in body frame (already in rad/s)
    float gyro[3] = {imu->gyro.x, imu->gyro.y, imu->gyro.z};

    // === Nominal State Propagation ===

    // Position update: p = p + v*dt + 0.5*a*dt^2
    eskf.pos[0] += eskf.vel[0] * dt + 0.5f * accel_world[0] * dt2;
    eskf.pos[1] += eskf.vel[1] * dt + 0.5f * accel_world[1] * dt2;
    eskf.pos[2] += eskf.vel[2] * dt + 0.5f * accel_world[2] * dt2;

    // Velocity update: v = v + a*dt
    eskf.vel[0] += accel_world[0] * dt;
    eskf.vel[1] += accel_world[1] * dt;
    eskf.vel[2] += accel_world[2] * dt;

    // Ground constraint (if below ground, reset)
    if (eskf.pos[2] < 0.0f)
    {
        eskf.pos[2] = 0.0f;
        if (eskf.vel[2] < 0.0f)
        {
            eskf.vel[2] = 0.0f;
        }
    }

    // Quaternion update: q = q * Δq(ω*dt)
    float dtheta[3] = {gyro[0] * dt, gyro[1] * dt, gyro[2] * dt};
    eskf_quat_t dq;
    rotation_vector_to_quat(dtheta, &dq);
    eskf_quat_t q_new;
    quat_multiply(&eskf.quat, &dq, &q_new);
    eskf.quat = q_new;
    quat_normalize(&eskf.quat);

    // === Error State Covariance Propagation ===
    // Fx is the Jacobian of the error state dynamics (9x9)
    // Fx = | I    dt*I   -0.5*dt^2*R*[a]_x |
    //      | 0    I      -dt*R*[a]_x       |
    //      | 0    0      exp([ω*dt]_x)^T   |

    // Build Fx matrix
    float Fx[ESKF_ERROR_DIM][ESKF_ERROR_DIM] = {0};

    // Top-left 3x3: I
    Fx[0][0] = 1.0f;
    Fx[1][1] = 1.0f;
    Fx[2][2] = 1.0f;

    // Top-middle 3x3: dt*I
    Fx[0][3] = dt;
    Fx[1][4] = dt;
    Fx[2][5] = dt;

    // Middle 3x3: I
    Fx[3][3] = 1.0f;
    Fx[4][4] = 1.0f;
    Fx[5][5] = 1.0f;

    // Skew-symmetric of acceleration
    float accel_skew[3][3];
    skew_symmetric(accel_body, accel_skew);

    // Top-right 3x3: -0.5*dt^2*R*[a]_x
    float R_accel_skew[3][3];
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            R_accel_skew[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                R_accel_skew[i][j] += R[i][k] * accel_skew[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Fx[i][6 + j] = -0.5f * dt2 * R_accel_skew[i][j];
            Fx[3 + i][6 + j] = -dt * R_accel_skew[i][j];
        }
    }

    // Bottom-right 3x3: exp([ω*dt]_x)^T
    float R_omega[3][3];
    rodrigues_exp(dtheta, R_omega);
    float R_omega_T[3][3];
    mat_transpose_3x3(R_omega, R_omega_T);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Fx[6 + i][6 + j] = R_omega_T[i][j];
        }
    }

    // Process noise covariance Qi (6x6)
    float Vi = W_ACCEL * W_ACCEL * dt2;  // Accel noise variance
    float Ti = W_GYRO * W_GYRO * dt2;    // Gyro noise variance

    // Fi*Qi*Fi^T contribution to P
    // Fi = | 0   0 |
    //      | I   0 |
    //      | 0   I |
    // So Fi*Qi*Fi^T adds Vi*I to P[3:6,3:6] and Ti*I to P[6:9,6:9]

    // P = Fx * P * Fx^T + Fi * Qi * Fi^T
    float P_new[ESKF_ERROR_DIM][ESKF_ERROR_DIM] = {0};

    // First compute Fx * P
    float FxP[ESKF_ERROR_DIM][ESKF_ERROR_DIM] = {0};
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            for (int k = 0; k < ESKF_ERROR_DIM; k++)
            {
                FxP[i][j] += Fx[i][k] * eskf.P[k][j];
            }
        }
    }

    // Then compute (Fx * P) * Fx^T
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            for (int k = 0; k < ESKF_ERROR_DIM; k++)
            {
                P_new[i][j] += FxP[i][k] * Fx[j][k]; // Fx^T[k][j] = Fx[j][k]
            }
        }
    }

    // Add process noise
    P_new[3][3] += Vi;
    P_new[4][4] += Vi;
    P_new[5][5] += Vi;
    P_new[6][6] += Ti;
    P_new[7][7] += Ti;
    P_new[8][8] += Ti;

    // Enforce symmetry and copy
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            eskf.P[i][j] = 0.5f * (P_new[i][j] + P_new[j][i]);
            // Enforce minimum covariance
            if (i == j && eskf.P[i][j] < MIN_COVARIANCE)
            {
                eskf.P[i][j] = MIN_COVARIANCE;
            }
        }
    }

    eskf.stats.imu_updates++;
}

/**
 * @brief UWB TWR correction step
 *
 * Measurement model: h(x) = ||p - p_anchor||
 * Innovation: y = z_meas - h(x_prior)
 * Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
 * State update: δx = K * y
 * Covariance update: P = (I - K*H) * P
 */
void sensor_fusion_uwb_update(const eskf_range_meas_t* range)
{
    if (!eskf.initialized || !eskf.anchors.configured)
    {
        return;
    }

    if (!range->valid || range->anchor_id >= ESKF_NUM_ANCHORS)
    {
        return;
    }

    // Get anchor position
    const eskf_vec3_t* anchor = &eskf.anchors.position[range->anchor_id];

    // Predicted range: ||p - p_anchor||
    float dx = eskf.pos[0] - anchor->x;
    float dy = eskf.pos[1] - anchor->y;
    float dz = eskf.pos[2] - anchor->z;
    float predicted_range = sqrtf(dx * dx + dy * dy + dz * dz);

    // Avoid division by zero
    if (predicted_range < 0.01f)
    {
        predicted_range = 0.01f;
    }

    // Innovation (measurement residual)
    float innovation = range->range_m - predicted_range;

    // Measurement Jacobian H (1x9)
    // H = [∂h/∂p, 0, 0] = [(p - p_anchor)^T / ||p - p_anchor||, 0, 0]
    float H[ESKF_ERROR_DIM];
    H[0] = dx / predicted_range;  // ∂h/∂x
    H[1] = dy / predicted_range;  // ∂h/∂y
    H[2] = dz / predicted_range;  // ∂h/∂z
    H[3] = 0.0f;                  // ∂h/∂vx
    H[4] = 0.0f;                  // ∂h/∂vy
    H[5] = 0.0f;                  // ∂h/∂vz
    H[6] = 0.0f;                  // ∂h/∂θx
    H[7] = 0.0f;                  // ∂h/∂θy
    H[8] = 0.0f;                  // ∂h/∂θz

    // Compute S = H * P * H^T + R (scalar)
    float PHt[ESKF_ERROR_DIM] = {0};
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            PHt[i] += eskf.P[i][j] * H[j];
        }
    }

    float S = STD_UWB_RANGE * STD_UWB_RANGE;  // Measurement variance
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        S += H[i] * PHt[i];
    }

    // Mahalanobis distance for outlier rejection
    float mahal_dist = sqrtf((innovation * innovation) / S);
    if (mahal_dist > OUTLIER_THRESHOLD)
    {
        eskf.stats.uwb_rejected++;
        return;
    }

    // Kalman gain K = P * H^T / S (9x1)
    float K[ESKF_ERROR_DIM];
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        K[i] = PHt[i] / S;
    }

    // Update error state: δx = K * innovation
    float dx_err[ESKF_ERROR_DIM];
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        dx_err[i] = K[i] * innovation;
    }

    // Inject error into nominal state
    // Position correction
    eskf.pos[0] += dx_err[0];
    eskf.pos[1] += dx_err[1];
    eskf.pos[2] += dx_err[2];

    // Velocity correction
    eskf.vel[0] += dx_err[3];
    eskf.vel[1] += dx_err[4];
    eskf.vel[2] += dx_err[5];

    // Attitude correction: q = q * Δq(δθ)
    float dtheta[3] = {dx_err[6], dx_err[7], dx_err[8]};
    eskf_quat_t dq;
    rotation_vector_to_quat(dtheta, &dq);
    eskf_quat_t q_corrected;
    quat_multiply(&eskf.quat, &dq, &q_corrected);
    eskf.quat = q_corrected;
    quat_normalize(&eskf.quat);

    // Update covariance: P = (I - K*H) * P
    float P_new[ESKF_ERROR_DIM][ESKF_ERROR_DIM];
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            float IKH = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
            P_new[i][j] = 0.0f;
            for (int k = 0; k < ESKF_ERROR_DIM; k++)
            {
                float IKH_ik = (i == k ? 1.0f : 0.0f) - K[i] * H[k];
                P_new[i][j] += IKH_ik * eskf.P[k][j];
            }
        }
    }

    // Enforce symmetry
    for (int i = 0; i < ESKF_ERROR_DIM; i++)
    {
        for (int j = 0; j < ESKF_ERROR_DIM; j++)
        {
            eskf.P[i][j] = 0.5f * (P_new[i][j] + P_new[j][i]);
            if (i == j && eskf.P[i][j] < MIN_COVARIANCE)
            {
                eskf.P[i][j] = MIN_COVARIANCE;
            }
        }
    }

    eskf.stats.uwb_updates++;
}

void sensor_fusion_get_state(eskf_state_t* state)
{
    state->position.x = eskf.pos[0];
    state->position.y = eskf.pos[1];
    state->position.z = eskf.pos[2];

    state->velocity.x = eskf.vel[0];
    state->velocity.y = eskf.vel[1];
    state->velocity.z = eskf.vel[2];

    state->attitude = eskf.quat;

    // Extract standard deviations from covariance diagonal
    state->position_std[0] = sqrtf(eskf.P[0][0]);
    state->position_std[1] = sqrtf(eskf.P[1][1]);
    state->position_std[2] = sqrtf(eskf.P[2][2]);

    state->velocity_std[0] = sqrtf(eskf.P[3][3]);
    state->velocity_std[1] = sqrtf(eskf.P[4][4]);
    state->velocity_std[2] = sqrtf(eskf.P[5][5]);

    state->initialized = eskf.initialized;
}

void sensor_fusion_get_stats(eskf_stats_t* stats)
{
    *stats = eskf.stats;
}

void sensor_fusion_reset(void)
{
    sensor_fusion_init();
}

bool sensor_fusion_is_initialized(void)
{
    return eskf.initialized && eskf.anchors.configured;
}

void sensor_fusion_isr(void)
{
    // Legacy placeholder - not used in new implementation
}
