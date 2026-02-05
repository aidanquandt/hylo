/**
 * @file kalman_core.h
 * @brief Extended Kalman Filter core for IMU/UWB sensor fusion
 *
 * Adapted from Crazyflie firmware kalman_core.h
 * Original authors: Michael Hamer, Mark Mueller
 * Original Copyright (C) Bitcraze AB, GPLv3
 *
 * Based on the papers:
 * - "Fusing ultra-wideband range measurements with accelerometers and rate 
 *    gyroscopes for quadrocopter state estimation" (Mueller, Hamer, ICRA 2015)
 * - "Covariance Correction Step for Kalman Filtering with an Attitude"
 *    (Mueller, Hehn, D'Andrea, AIAA 2016)
 *
 * State vector (9 dimensions):
 * - X, Y, Z: Position in world frame (m)
 * - PX, PY, PZ: Velocity in body frame (m/s)
 * - D0, D1, D2: Attitude error (rad)
 *
 * Attitude is maintained as a quaternion separately from the error-state.
 */

#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sf_math.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

/** Number of states in the Kalman filter */
#define KC_STATE_DIM 9

/** Maximum number of UWB anchors supported */
#define KC_MAX_ANCHORS 8

/*---------------------------------------------------------------------------
 * Type definitions
 *---------------------------------------------------------------------------*/

/**
 * @brief Indexes to access the state vector
 */
typedef enum {
    KC_STATE_X = 0,   /**< Position X (world frame, m) */
    KC_STATE_Y = 1,   /**< Position Y (world frame, m) */
    KC_STATE_Z = 2,   /**< Position Z (world frame, m) */
    KC_STATE_PX = 3,  /**< Velocity X (body frame, m/s) */
    KC_STATE_PY = 4,  /**< Velocity Y (body frame, m/s) */
    KC_STATE_PZ = 5,  /**< Velocity Z (body frame, m/s) */
    KC_STATE_D0 = 6,  /**< Attitude error roll (rad) */
    KC_STATE_D1 = 7,  /**< Attitude error pitch (rad) */
    KC_STATE_D2 = 8   /**< Attitude error yaw (rad) */
} kalmanCoreStateIdx_t;

/**
 * @brief 3-axis float vector (for acc/gyro data)
 */
typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

/**
 * @brief UWB distance measurement to an anchor
 */
typedef struct {
    float x;         /**< Anchor X position (m) */
    float y;         /**< Anchor Y position (m) */
    float z;         /**< Anchor Z position (m) */
    float distance;  /**< Measured distance (m) */
    float stdDev;    /**< Measurement standard deviation (m) */
    uint8_t anchorId; /**< Anchor identifier */
} distanceMeasurement_t;

/**
 * @brief Kalman filter tuning parameters
 */
typedef struct {
    /* Initial state variances */
    float stdDevInitialPosition_xy;      /**< Initial XY position std dev (m) */
    float stdDevInitialPosition_z;       /**< Initial Z position std dev (m) */
    float stdDevInitialVelocity;         /**< Initial velocity std dev (m/s) */
    float stdDevInitialAttitude_rollpitch; /**< Initial roll/pitch std dev (rad) */
    float stdDevInitialAttitude_yaw;     /**< Initial yaw std dev (rad) */

    /* Process noise */
    float procNoiseAcc_xy;    /**< Accelerometer XY process noise */
    float procNoiseAcc_z;     /**< Accelerometer Z process noise */
    float procNoiseVel;       /**< Velocity process noise */
    float procNoisePos;       /**< Position process noise */
    float procNoiseAtt;       /**< Attitude process noise */

    /* Measurement noise */
    float measNoiseGyro_rollpitch; /**< Gyro roll/pitch noise (rad/s) */
    float measNoiseGyro_yaw;       /**< Gyro yaw noise (rad/s) */

    /* Initial state */
    float initialX;   /**< Initial X position (m) */
    float initialY;   /**< Initial Y position (m) */
    float initialZ;   /**< Initial Z position (m) */
    float initialYaw; /**< Initial yaw angle (rad) */
} kalmanCoreParams_t;

/**
 * @brief Main Kalman filter data structure
 */
typedef struct {
    /**
     * State vector [X, Y, Z, PX, PY, PZ, D0, D1, D2]
     * - X, Y, Z: Position in world frame (m)
     * - PX, PY, PZ: Velocity in body frame (m/s)  
     * - D0, D1, D2: Attitude error (rad)
     */
    float S[KC_STATE_DIM];

    /** Attitude quaternion [w, x, y, z] */
    float q[4];

    /** Rotation matrix (body to world) */
    float R[3][3];

    /** Covariance matrix (9x9) - must be 4-byte aligned for ARM DSP */
    __attribute__((aligned(4))) float P[KC_STATE_DIM][KC_STATE_DIM];

    /** ARM matrix instance for covariance */
    arm_matrix_instance_f32 Pm;

    /** Initial quaternion for attitude reversion */
    float initialQuaternion[4];

    /** Flag indicating state was updated and needs finalization */
    bool isUpdated;

    /** Timestamp of last prediction (ms) */
    uint32_t lastPredictionMs;

    /** Timestamp of last process noise update (ms) */
    uint32_t lastProcessNoiseUpdateMs;
} kalmanCoreData_t;

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Set default parameter values
 * @param params Pointer to parameters structure to initialize
 */
void kalmanCoreDefaultParams(kalmanCoreParams_t* params);

/**
 * @brief Initialize the Kalman filter
 * @param kf Pointer to filter data structure
 * @param params Pointer to filter parameters
 * @param nowMs Current time in milliseconds
 */
void kalmanCoreInit(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, uint32_t nowMs);

/**
 * @brief Predict step using IMU data
 * @param kf Pointer to filter data
 * @param params Pointer to filter parameters
 * @param acc Accelerometer reading (m/s², body frame)
 * @param gyro Gyroscope reading (rad/s, body frame)
 * @param nowMs Current time in milliseconds
 */
void kalmanCorePredict(kalmanCoreData_t* kf, 
                       const kalmanCoreParams_t* params,
                       Axis3f* acc, 
                       Axis3f* gyro, 
                       uint32_t nowMs);

/**
 * @brief Add process noise to covariance matrix
 * @param kf Pointer to filter data
 * @param params Pointer to filter parameters
 * @param nowMs Current time in milliseconds
 */
void kalmanCoreAddProcessNoise(kalmanCoreData_t* kf, 
                               const kalmanCoreParams_t* params, 
                               uint32_t nowMs);

/**
 * @brief Scalar Kalman update step
 * @param kf Pointer to filter data
 * @param Hm Measurement Jacobian matrix (1 x KC_STATE_DIM)
 * @param error Innovation (measurement - prediction)
 * @param stdMeasNoise Measurement standard deviation
 */
void kalmanCoreScalarUpdate(kalmanCoreData_t* kf, 
                            arm_matrix_instance_f32* Hm, 
                            float error, 
                            float stdMeasNoise);

/**
 * @brief Kalman update with pre-computed weighted matrices
 * @param kf Pointer to filter data
 * @param Hm Measurement Jacobian matrix
 * @param Km Kalman gain matrix
 * @param P_w_m Weighted covariance matrix
 * @param error Innovation
 */
void kalmanCoreUpdateWithPKE(kalmanCoreData_t* kf, 
                             arm_matrix_instance_f32* Hm, 
                             arm_matrix_instance_f32* Km, 
                             arm_matrix_instance_f32* P_w_m, 
                             float error);

/**
 * @brief Update with UWB TWR distance measurement (standard)
 * @param kf Pointer to filter data
 * @param d Distance measurement
 */
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* kf, distanceMeasurement_t* d);

/**
 * @brief Update with UWB TWR distance measurement (robust M-estimation)
 * @param kf Pointer to filter data
 * @param d Distance measurement
 */
void kalmanCoreRobustUpdateWithDistance(kalmanCoreData_t* kf, distanceMeasurement_t* d);

/**
 * @brief Finalize state update (incorporate attitude error into quaternion)
 * @param kf Pointer to filter data
 * @return true if state was finalized, false if no update was needed
 */
bool kalmanCoreFinalize(kalmanCoreData_t* kf);

/**
 * @brief Get position from state
 * @param kf Pointer to filter data
 * @param x Output X position (m)
 * @param y Output Y position (m)
 * @param z Output Z position (m)
 */
void kalmanCoreGetPosition(const kalmanCoreData_t* kf, float* x, float* y, float* z);

/**
 * @brief Get velocity in world frame
 * @param kf Pointer to filter data
 * @param vx Output X velocity (m/s)
 * @param vy Output Y velocity (m/s)
 * @param vz Output Z velocity (m/s)
 */
void kalmanCoreGetVelocity(const kalmanCoreData_t* kf, float* vx, float* vy, float* vz);

/**
 * @brief Get attitude as Euler angles
 * @param kf Pointer to filter data
 * @param roll Output roll angle (rad)
 * @param pitch Output pitch angle (rad)
 * @param yaw Output yaw angle (rad)
 */
void kalmanCoreGetAttitude(const kalmanCoreData_t* kf, float* roll, float* pitch, float* yaw);

/**
 * @brief Get attitude as quaternion
 * @param kf Pointer to filter data
 * @param qw Output quaternion w component
 * @param qx Output quaternion x component
 * @param qy Output quaternion y component
 * @param qz Output quaternion z component
 */
void kalmanCoreGetQuaternion(const kalmanCoreData_t* kf, 
                             float* qw, float* qx, float* qy, float* qz);

