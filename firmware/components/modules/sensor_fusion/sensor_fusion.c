/**
 * @file sensor_fusion.c
 * @brief Sensor Fusion Module - Implementation
 *
 * This module wraps the Kalman filter core to provide a clean API for
 * IMU/UWB sensor fusion on the STM32H7 platform.
 *
 * Adapted from Crazyflie firmware (GPLv3) for the Hylo project.
 */

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "kalman_core.h"

/*---------------------------------------------------------------------------
 * Private defines
 *---------------------------------------------------------------------------*/

/** Default UWB measurement standard deviation (m) */
#define DEFAULT_UWB_STD_DEV (0.15f)

/** Minimum number of updates before state is considered valid */
#define MIN_UPDATES_FOR_VALID (10)

/*---------------------------------------------------------------------------
 * Private variables
 *---------------------------------------------------------------------------*/

/** Kalman filter data structure */
static kalmanCoreData_t s_kf_data;

/** Kalman filter parameters */
static kalmanCoreParams_t s_kf_params;

/** Anchor configurations */
static sf_anchor_config_t s_anchors[SF_MAX_ANCHORS];

/** Default UWB standard deviation */
static float s_default_uwb_std_dev = DEFAULT_UWB_STD_DEV;

/** Flag to use robust M-estimation */
static bool s_use_robust_twr = false;

/** Update counter for validity check */
static uint32_t s_update_count = 0;

/** Last update timestamp */
static uint32_t s_last_timestamp_ms = 0;

/** Initialization flag */
static bool s_initialized = false;

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
static void set_default_anchors(void);
static void finalize_if_needed(void);

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/

void sensor_fusion_init(void)
{
    /* Use default configuration */
    sensor_fusion_config_t default_config = {
        .initial_x = 0.0f,
        .initial_y = 0.0f,
        .initial_z = 0.0f,
        .initial_yaw = 0.0f,
        .proc_noise_acc_xy = 0.5f,
        .proc_noise_acc_z = 1.0f,
        .uwb_std_dev = DEFAULT_UWB_STD_DEV,
        .use_robust_twr = false
    };
    
    sensor_fusion_init_with_config(&default_config);
}

void sensor_fusion_init_with_config(const sensor_fusion_config_t* config)
{
    /* Initialize filter parameters with defaults */
    kalmanCoreDefaultParams(&s_kf_params);
    
    /* Apply custom configuration */
    s_kf_params.initialX = config->initial_x;
    s_kf_params.initialY = config->initial_y;
    s_kf_params.initialZ = config->initial_z;
    s_kf_params.initialYaw = config->initial_yaw;
    s_kf_params.procNoiseAcc_xy = config->proc_noise_acc_xy;
    s_kf_params.procNoiseAcc_z = config->proc_noise_acc_z;
    
    /* Store UWB configuration */
    s_default_uwb_std_dev = config->uwb_std_dev;
    s_use_robust_twr = config->use_robust_twr;
    
    /* Initialize the Kalman filter core */
    kalmanCoreInit(&s_kf_data, &s_kf_params, 0);
    
    /* Set default anchor positions */
    set_default_anchors();
    
    /* Reset counters */
    s_update_count = 0;
    s_last_timestamp_ms = 0;
    s_initialized = true;
}

void sensor_fusion_reset(void)
{
    if (!s_initialized) {
        sensor_fusion_init();
        return;
    }
    
    /* Reinitialize with current parameters */
    kalmanCoreInit(&s_kf_data, &s_kf_params, s_last_timestamp_ms);
    s_update_count = 0;
}

bool sensor_fusion_set_anchor_position(uint8_t anchor_id, float x, float y, float z)
{
    if (anchor_id >= SF_MAX_ANCHORS) {
        return false;
    }
    
    s_anchors[anchor_id].x = x;
    s_anchors[anchor_id].y = y;
    s_anchors[anchor_id].z = z;
    s_anchors[anchor_id].enabled = true;
    
    return true;
}

bool sensor_fusion_set_anchor_enabled(uint8_t anchor_id, bool enabled)
{
    if (anchor_id >= SF_MAX_ANCHORS) {
        return false;
    }
    
    s_anchors[anchor_id].enabled = enabled;
    return true;
}

void sensor_fusion_predict_imu(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint32_t timestamp_ms)
{
    if (!s_initialized) {
        sensor_fusion_init();
    }
    
    /* Pack IMU data into Axis3f structures */
    Axis3f acc = {.x = ax, .y = ay, .z = az};
    Axis3f gyro = {.x = gx, .y = gy, .z = gz};
    
    /* Run prediction step */
    kalmanCorePredict(&s_kf_data, &s_kf_params, &acc, &gyro, timestamp_ms);
    
    /* Add process noise */
    kalmanCoreAddProcessNoise(&s_kf_data, &s_kf_params, timestamp_ms);
    
    /* Finalize to incorporate attitude corrections */
    finalize_if_needed();
    
    s_last_timestamp_ms = timestamp_ms;
}

void sensor_fusion_update_twr(uint8_t anchor_id, float distance, uint32_t timestamp_ms)
{
    sensor_fusion_update_twr_with_stddev(anchor_id, distance, s_default_uwb_std_dev, timestamp_ms);
}

void sensor_fusion_update_twr_with_stddev(uint8_t anchor_id, float distance,
                                          float std_dev, uint32_t timestamp_ms)
{
    if (!s_initialized) {
        sensor_fusion_init();
    }
    
    /* Validate anchor ID */
    if (anchor_id >= SF_MAX_ANCHORS) {
        return;
    }
    
    /* Check if anchor is enabled */
    if (!s_anchors[anchor_id].enabled) {
        return;
    }
    
    /* Validate distance measurement */
    if (distance <= 0.0f || isnan(distance)) {
        return;
    }
    
    /* Construct distance measurement */
    distanceMeasurement_t d = {
        .x = s_anchors[anchor_id].x,
        .y = s_anchors[anchor_id].y,
        .z = s_anchors[anchor_id].z,
        .distance = distance,
        .stdDev = std_dev,
        .anchorId = anchor_id
    };
    
    /* Run update step */
    if (s_use_robust_twr) {
        kalmanCoreRobustUpdateWithDistance(&s_kf_data, &d);
    } else {
        kalmanCoreUpdateWithDistance(&s_kf_data, &d);
    }
    
    /* Finalize to incorporate corrections */
    finalize_if_needed();
    
    s_update_count++;
    s_last_timestamp_ms = timestamp_ms;
}

void sensor_fusion_get_state(sensor_fusion_state_t* state)
{
    if (state == NULL) {
        return;
    }
    
    if (!s_initialized) {
        memset(state, 0, sizeof(sensor_fusion_state_t));
        state->valid = false;
        return;
    }
    
    /* Get position */
    kalmanCoreGetPosition(&s_kf_data, 
                          &state->position.x, 
                          &state->position.y, 
                          &state->position.z);
    
    /* Get velocity */
    kalmanCoreGetVelocity(&s_kf_data,
                          &state->velocity.x,
                          &state->velocity.y,
                          &state->velocity.z);
    
    /* Get attitude */
    kalmanCoreGetAttitude(&s_kf_data,
                          &state->attitude.roll,
                          &state->attitude.pitch,
                          &state->attitude.yaw);
    
    /* Get quaternion */
    kalmanCoreGetQuaternion(&s_kf_data,
                            &state->quaternion.w,
                            &state->quaternion.x,
                            &state->quaternion.y,
                            &state->quaternion.z);
    
    state->timestamp_ms = s_last_timestamp_ms;
    state->valid = sensor_fusion_is_valid();
}

void sensor_fusion_get_position(float* x, float* y, float* z)
{
    if (!s_initialized) {
        *x = *y = *z = 0.0f;
        return;
    }
    
    kalmanCoreGetPosition(&s_kf_data, x, y, z);
}

void sensor_fusion_get_velocity(float* vx, float* vy, float* vz)
{
    if (!s_initialized) {
        *vx = *vy = *vz = 0.0f;
        return;
    }
    
    kalmanCoreGetVelocity(&s_kf_data, vx, vy, vz);
}

void sensor_fusion_get_attitude(float* roll, float* pitch, float* yaw)
{
    if (!s_initialized) {
        *roll = *pitch = *yaw = 0.0f;
        return;
    }
    
    kalmanCoreGetAttitude(&s_kf_data, roll, pitch, yaw);
}

void sensor_fusion_get_quaternion(float* qw, float* qx, float* qy, float* qz)
{
    if (!s_initialized) {
        *qw = 1.0f;
        *qx = *qy = *qz = 0.0f;
        return;
    }
    
    kalmanCoreGetQuaternion(&s_kf_data, qw, qx, qy, qz);
}

bool sensor_fusion_is_valid(void)
{
    if (!s_initialized) {
        return false;
    }
    
    /* Check if we've had enough updates */
    if (s_update_count < MIN_UPDATES_FOR_VALID) {
        return false;
    }
    
    /* Check for NaN in position state */
    float x, y, z;
    kalmanCoreGetPosition(&s_kf_data, &x, &y, &z);
    
    if (isnan(x) || isnan(y) || isnan(z)) {
        return false;
    }
    
    /* Check for unreasonable position values */
    if (fabsf(x) > 1000.0f || fabsf(y) > 1000.0f || fabsf(z) > 1000.0f) {
        return false;
    }
    
    return true;
}

void sensor_fusion_set_robust_twr(bool enable)
{
    s_use_robust_twr = enable;
}

void sensor_fusion_isr(void)
{
    /* Reserved for future use - interrupt-driven updates */
}

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Set default anchor positions for a typical 4-anchor setup
 *
 * Default configuration assumes a rectangular room with anchors at corners.
 * Users should call sensor_fusion_set_anchor_position() to override.
 */
static void set_default_anchors(void)
{
    /* Initialize all anchors as disabled */
    for (int i = 0; i < SF_MAX_ANCHORS; i++) {
        s_anchors[i].x = 0.0f;
        s_anchors[i].y = 0.0f;
        s_anchors[i].z = 0.0f;
        s_anchors[i].enabled = false;
    }
    
    /* Set up default 4-anchor configuration (5m x 5m room at 2m height) */
    /* Anchor 0: Origin corner */
    s_anchors[0].x = 0.0f;
    s_anchors[0].y = 0.0f;
    s_anchors[0].z = 2.0f;
    s_anchors[0].enabled = true;
    
    /* Anchor 1: X-axis corner */
    s_anchors[1].x = 5.0f;
    s_anchors[1].y = 0.0f;
    s_anchors[1].z = 2.0f;
    s_anchors[1].enabled = true;
    
    /* Anchor 2: Y-axis corner */
    s_anchors[2].x = 0.0f;
    s_anchors[2].y = 5.0f;
    s_anchors[2].z = 2.0f;
    s_anchors[2].enabled = true;
    
    /* Anchor 3: Diagonal corner */
    s_anchors[3].x = 5.0f;
    s_anchors[3].y = 5.0f;
    s_anchors[3].z = 2.0f;
    s_anchors[3].enabled = true;
}

/**
 * @brief Finalize state update if needed
 *
 * This incorporates attitude error into the main quaternion and
 * updates the rotation matrix.
 */
static void finalize_if_needed(void)
{
    kalmanCoreFinalize(&s_kf_data);
}
