/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "kalman_core.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private defines
 *---------------------------------------------------------------------------*/

/** Maximum covariance bound (prevents numerical instability) */
#define MAX_COVARIANCE (100.0f)

/** Minimum covariance bound (prevents filter divergence) */
#define MIN_COVARIANCE (1e-6f)

/** Small epsilon to prevent division by zero */
#define EPS SF_EPS

/** Upper bound for robust filter matrix elements */
#define ROBUST_UPPER_BOUND (100.0f)

/** Lower bound for robust filter matrix elements */
#define ROBUST_LOWER_BOUND (-100.0f)

/** Maximum iterations for robust M-estimation */
#define ROBUST_MAX_ITER (2)

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
static void predictDt(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, Axis3f* acc,
                      Axis3f* gyro, float dt);
static void addProcessNoiseDt(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, float dt);
static void enforceSymmetryAndBounds(kalmanCoreData_t* kf);

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/

void kalmanCoreDefaultParams(kalmanCoreParams_t* params)
{
    /* Initial state variances */
    params->stdDevInitialPosition_xy        = 100.0f; /* Large uncertainty in XY */
    params->stdDevInitialPosition_z         = 1.0f;   /* Smaller uncertainty in Z */
    params->stdDevInitialVelocity           = 0.01f;  /* Know we start stationary */
    params->stdDevInitialAttitude_rollpitch = 0.01f;  /* Roughly flat */
    params->stdDevInitialAttitude_yaw       = 0.01f;  /* Unknown heading */

    /* Process noise - tune based on IMU quality */
    /* These values model the uncertainty in IMU measurements and allow the filter */
    /* to appropriately weight IMU predictions vs UWB measurements */
    params->procNoiseAcc_xy = 0.5f;    /* Accelerometer noise XY (m/s^2) */
    params->procNoiseAcc_z  = 1.0f;    /* Accelerometer noise Z (m/s^2) */
    params->procNoiseVel    = 0.1f;    /* Velocity random walk (m/s) - tuned for human walking pace */
    params->procNoisePos    = 0.01f;   /* Position random walk (m) - tuned for human walking pace */
    params->procNoiseAtt    = 0.0001f; /* Attitude drift (rad) - Small but non-zero */

    /* Measurement noise */
    params->measNoiseGyro_rollpitch = 0.1f; /* rad/s */
    params->measNoiseGyro_yaw       = 0.1f; /* rad/s */

    /* Initial state */
    params->initialX   = 1.44f;
    params->initialY   = 3.05f;
    params->initialZ   = 0.80f;
    params->initialYaw = 0.0f;
}

void kalmanCoreInit(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, uint32_t nowMs)
{
    /* Reset all data to zero */
    memset(kf, 0, sizeof(kalmanCoreData_t));

    /* Initialize position state */
    kf->S[KC_STATE_X] = params->initialX;
    kf->S[KC_STATE_Y] = params->initialY;
    kf->S[KC_STATE_Z] = params->initialZ;
    /* Velocity and attitude error states start at zero */

    /* Initialize attitude quaternion from initial yaw */
    kf->initialQuaternion[0] = arm_cos_f32(params->initialYaw / 2.0f); /* w */
    kf->initialQuaternion[1] = 0.0f;                                   /* x */
    kf->initialQuaternion[2] = 0.0f;                                   /* y */
    kf->initialQuaternion[3] = arm_sin_f32(params->initialYaw / 2.0f); /* z */

    for (int i = 0; i < 4; i++)
    {
        kf->q[i] = kf->initialQuaternion[i];
    }

    /* Initialize rotation matrix to identity */
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf->R[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    /* Initialize covariance matrix */
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        for (int j = 0; j < KC_STATE_DIM; j++)
        {
            kf->P[i][j] = 0.0f;
        }
    }

    /* Set initial state variances (diagonal elements) */
    kf->P[KC_STATE_X][KC_STATE_X] = powf(params->stdDevInitialPosition_xy, 2);
    kf->P[KC_STATE_Y][KC_STATE_Y] = powf(params->stdDevInitialPosition_xy, 2);
    kf->P[KC_STATE_Z][KC_STATE_Z] = powf(params->stdDevInitialPosition_z, 2);

    kf->P[KC_STATE_PX][KC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
    kf->P[KC_STATE_PY][KC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
    kf->P[KC_STATE_PZ][KC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

    kf->P[KC_STATE_D0][KC_STATE_D0] = powf(params->stdDevInitialAttitude_rollpitch, 2);
    kf->P[KC_STATE_D1][KC_STATE_D1] = powf(params->stdDevInitialAttitude_rollpitch, 2);
    kf->P[KC_STATE_D2][KC_STATE_D2] = powf(params->stdDevInitialAttitude_yaw, 2);

    /* Initialize ARM matrix instance for covariance */
    kf->Pm.numRows = KC_STATE_DIM;
    kf->Pm.numCols = KC_STATE_DIM;
    kf->Pm.pData   = (float*)kf->P;

    kf->isUpdated                = false;
    kf->lastPredictionMs         = nowMs;
    kf->lastProcessNoiseUpdateMs = nowMs;
}

void kalmanCorePredict(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, Axis3f* acc,
                       Axis3f* gyro, uint32_t nowMs)
{
    float dt = (nowMs - kf->lastPredictionMs) / 1000.0f;

    if (dt > 0.0f && dt < 1.0f)
    { /* Sanity check on dt */
        predictDt(kf, params, acc, gyro, dt);
    }

    kf->lastPredictionMs = nowMs;
}

void kalmanCoreAddProcessNoise(kalmanCoreData_t* kf, const kalmanCoreParams_t* params,
                               uint32_t nowMs)
{
    float dt = (nowMs - kf->lastProcessNoiseUpdateMs) / 1000.0f;

    if (dt > 0.0f)
    {
        addProcessNoiseDt(kf, params, dt);
        kf->lastProcessNoiseUpdateMs = nowMs;
    }
}

void kalmanCoreScalarUpdate(kalmanCoreData_t* kf, arm_matrix_instance_f32* Hm, float error,
                            float stdMeasNoise)
{
    /* Kalman gain as a column vector */
    static float K[KC_STATE_DIM];
    static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float*)K};

    /* Temporary matrices for covariance updates */
    static __attribute__((aligned(4))) float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

    static __attribute__((aligned(4))) float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

    static __attribute__((aligned(4))) float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

    static __attribute__((aligned(4))) float HTd[KC_STATE_DIM];
    static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

    static __attribute__((aligned(4))) float PHTd[KC_STATE_DIM];
    static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

    SF_ASSERT(Hm->numRows == 1);
    SF_ASSERT(Hm->numCols == KC_STATE_DIM);

    /* ====== INNOVATION COVARIANCE ====== */
    mat_trans(Hm, &HTm);            /* H' */
    mat_mult(&kf->Pm, &HTm, &PHTm); /* P * H' */

    float R    = stdMeasNoise * stdMeasNoise;
    float HPHR = R; /* H * P * H' + R */

    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        HPHR += Hm->pData[i] * PHTd[i];
    }

    SF_ASSERT(!isnan(HPHR));

    /* ====== MEASUREMENT UPDATE ====== */
    /* Calculate Kalman gain and perform state update */
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        K[i]     = PHTd[i] / HPHR;          /* K = P * H' / (H * P * H' + R) */
        kf->S[i] = kf->S[i] + K[i] * error; /* State update */
    }

    /* ====== COVARIANCE UPDATE ====== */
    /* Joseph form update: P = (I - K*H) * P * (I - K*H)' + K*R*K' */
    mat_mult(&Km, Hm, &tmpNN1m); /* K * H */

    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        tmpNN1d[KC_STATE_DIM * i + i] -= 1.0f; /* K*H - I */
    }

    mat_trans(&tmpNN1m, &tmpNN2m);         /* (K*H - I)' */
    mat_mult(&tmpNN1m, &kf->Pm, &tmpNN3m); /* (K*H - I) * P */
    mat_mult(&tmpNN3m, &tmpNN2m, &kf->Pm); /* (K*H - I) * P * (K*H - I)' */

    /* Add measurement variance and ensure boundedness and symmetry */
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        for (int j = i; j < KC_STATE_DIM; j++)
        {
            float v = K[i] * R * K[j];
            float p = 0.5f * kf->P[i][j] + 0.5f * kf->P[j][i] + v;

            if (isnan(p) || p > MAX_COVARIANCE)
            {
                kf->P[i][j] = kf->P[j][i] = MAX_COVARIANCE;
            }
            else if (i == j && p < MIN_COVARIANCE)
            {
                kf->P[i][j] = kf->P[j][i] = MIN_COVARIANCE;
            }
            else
            {
                kf->P[i][j] = kf->P[j][i] = p;
            }
        }
    }

    kf->isUpdated = true;
}

void kalmanCoreUpdateWithPKE(kalmanCoreData_t* kf, arm_matrix_instance_f32* Hm,
                             arm_matrix_instance_f32* Km, arm_matrix_instance_f32* P_w_m,
                             float error)
{
    static float tmpNN1d[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, (float*)tmpNN1d};

    /* State update */
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        kf->S[i] = kf->S[i] + Km->pData[i] * error;
    }

    /* Covariance update: P = (I - K*H) * P_w */
    mat_mult(Km, Hm, &tmpNN1m);           /* K*H */
    mat_scale(&tmpNN1m, -1.0f, &tmpNN1m); /* -K*H */

    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        tmpNN1d[i][i] = 1.0f + tmpNN1d[i][i]; /* I - K*H */
    }

    float Ppo[KC_STATE_DIM][KC_STATE_DIM] = {0};
    arm_matrix_instance_f32 Ppom          = {KC_STATE_DIM, KC_STATE_DIM, (float*)Ppo};

    mat_mult(&tmpNN1m, P_w_m, &Ppom); /* P = (I - K*H) * P_w */
    memcpy(kf->P, Ppo, sizeof(kf->P));

    enforceSymmetryAndBounds(kf);
    kf->isUpdated = true;
}

void kalmanCoreUpdateWithDistance(kalmanCoreData_t* kf, distanceMeasurement_t* d)
{
    /* Measurement Jacobian */
    float h[KC_STATE_DIM]     = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

    float dx = kf->S[KC_STATE_X] - d->x;
    float dy = kf->S[KC_STATE_Y] - d->y;
    float dz = kf->S[KC_STATE_Z] - d->z;

    float predictedDistance = arm_sqrt(dx * dx + dy * dy + dz * dz);

    if (predictedDistance > EPS)
    {
        /* dz/dX = (X - x_anchor) / distance */
        h[KC_STATE_X] = dx / predictedDistance;
        h[KC_STATE_Y] = dy / predictedDistance;
        h[KC_STATE_Z] = dz / predictedDistance;
    }
    else
    {
        /* Avoid divide by zero - assume measurement is mostly in X direction */
        h[KC_STATE_X] = 1.0f;
        h[KC_STATE_Y] = 0.0f;
        h[KC_STATE_Z] = 0.0f;
    }

    float error = d->distance - predictedDistance;
    kalmanCoreScalarUpdate(kf, &H, error, d->stdDev);
}

/**
 * @brief GM weight function for UWB measurements (robust M-estimation)
 */
static void GM_UWB(float e, float* GM_e)
{
    float sigma = 1.5f;
    float GM_dn = sigma + e * e;
    *GM_e       = (sigma * sigma) / (GM_dn * GM_dn);
}

/**
 * @brief GM weight function for state (robust M-estimation)
 */
static void GM_state(float e, float* GM_e)
{
    float sigma = 2.0f;
    float GM_dn = sigma + e * e;
    *GM_e       = (sigma * sigma) / (GM_dn * GM_dn);
}

/**
 * @brief Cholesky decomposition for positive semi-definite matrix
 */
static void Cholesky_Decomposition(int n, float matrix[n][n], float lower[n][n])
{
    memset(lower, 0, n * n * sizeof(float));

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            float sum = 0.0f;

            if (j == i)
            {
                /* Diagonal elements */
                for (int k = 0; k < j; k++)
                {
                    sum += powf(lower[j][k], 2);
                }
                float val   = matrix[j][j] - sum;
                lower[j][j] = (val > 0.0f) ? sqrtf(val) : 0.0f;
            }
            else
            {
                /* Off-diagonal elements */
                for (int k = 0; k < j; k++)
                {
                    sum += lower[i][k] * lower[j][k];
                }
                if (fabsf(lower[j][j]) > EPS)
                {
                    lower[i][j] = (matrix[i][j] - sum) / lower[j][j];
                }
            }
        }
    }
}

void kalmanCoreRobustUpdateWithDistance(kalmanCoreData_t* kf, distanceMeasurement_t* d)
{
    float dx               = kf->S[KC_STATE_X] - d->x;
    float dy               = kf->S[KC_STATE_Y] - d->y;
    float dz               = kf->S[KC_STATE_Z] - d->z;
    float measuredDistance = d->distance;

    float predictedDistance = arm_sqrt(dx * dx + dy * dy + dz * dz);
    float error_check       = measuredDistance - predictedDistance;

    /* Matrix definitions */
    static float P_chol[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Pc_m = {KC_STATE_DIM, KC_STATE_DIM, (float*)P_chol};
    static float Pc_tran[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Pc_tran_m = {KC_STATE_DIM, KC_STATE_DIM, (float*)Pc_tran};

    float h[KC_STATE_DIM]     = {0};
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

    static float Kw[KC_STATE_DIM];
    static arm_matrix_instance_f32 Kwm = {KC_STATE_DIM, 1, (float*)Kw};

    static float e_x[KC_STATE_DIM];
    static arm_matrix_instance_f32 e_x_m = {KC_STATE_DIM, 1, e_x};

    static float Pc_inv[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Pc_inv_m = {KC_STATE_DIM, KC_STATE_DIM, (float*)Pc_inv};

    static float wx_inv[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 wx_invm = {KC_STATE_DIM, KC_STATE_DIM, (float*)wx_inv};

    static float tmp1[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 tmp1m = {KC_STATE_DIM, KC_STATE_DIM, (float*)tmp1};

    static float Pc_w_inv[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Pc_w_invm = {KC_STATE_DIM, KC_STATE_DIM, (float*)Pc_w_inv};

    static float P_w[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 P_w_m = {KC_STATE_DIM, KC_STATE_DIM, (float*)P_w};

    static float HTd[KC_STATE_DIM];
    static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

    static float PHTd[KC_STATE_DIM];
    static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

    /* Initialization */
    static float x_err[KC_STATE_DIM]      = {0};
    static arm_matrix_instance_f32 x_errm = {KC_STATE_DIM, 1, x_err};
    static float X_state[KC_STATE_DIM]    = {0};

    float P_iter[KC_STATE_DIM][KC_STATE_DIM];
    memcpy(P_iter, kf->P, sizeof(P_iter));

    float R_iter = d->stdDev * d->stdDev;
    memcpy(X_state, kf->S, sizeof(X_state));
    memset(x_err, 0, sizeof(x_err));
    memset(wx_inv, 0, sizeof(wx_inv));

    /* Iterative robust update */
    for (int iter = 0; iter < ROBUST_MAX_ITER; iter++)
    {
        /* Cholesky decomposition of prior covariance */
        Cholesky_Decomposition(KC_STATE_DIM, P_iter, P_chol);
        mat_trans(&Pc_m, &Pc_tran_m);

        float R_chol = sqrtf(R_iter);

        /* Update state and construct H matrix */
        float x_iter = X_state[KC_STATE_X];
        float y_iter = X_state[KC_STATE_Y];
        float z_iter = X_state[KC_STATE_Z];

        dx = x_iter - d->x;
        dy = y_iter - d->y;
        dz = z_iter - d->z;

        float predicted_iter = arm_sqrt(dx * dx + dy * dy + dz * dz);
        float error_iter     = measuredDistance - predicted_iter;
        float e_y            = error_iter;

        if (predicted_iter > EPS)
        {
            h[KC_STATE_X] = dx / predicted_iter;
            h[KC_STATE_Y] = dy / predicted_iter;
            h[KC_STATE_Z] = dz / predicted_iter;
        }
        else
        {
            h[KC_STATE_X] = 1.0f;
            h[KC_STATE_Y] = 0.0f;
            h[KC_STATE_Z] = 0.0f;
        }

        /* Zero out velocity and attitude components of H */
        for (int i = KC_STATE_PX; i < KC_STATE_DIM; i++)
        {
            h[i] = 0.0f;
        }

        /* Normalize error by measurement noise */
        if (fabsf(R_chol) < 0.0001f)
        {
            e_y = error_iter / 0.0001f;
        }
        else
        {
            e_y = error_iter / R_chol;
        }

        /* Ensure P_chol is numerically stable */
        for (int col = 0; col < KC_STATE_DIM; col++)
        {
            for (int row = col; row < KC_STATE_DIM; row++)
            {
                if (isnan(P_chol[row][col]) || P_chol[row][col] > ROBUST_UPPER_BOUND)
                {
                    P_chol[row][col] = ROBUST_UPPER_BOUND;
                }
                else if (row != col && P_chol[row][col] < ROBUST_LOWER_BOUND)
                {
                    P_chol[row][col] = ROBUST_LOWER_BOUND;
                }
                else if (row == col && P_chol[row][col] < 0.0f)
                {
                    P_chol[row][col] = 0.0f;
                }
            }
        }

        /* Add small diagonal for numerical stability */
        float dummy_value = 1e-9f;
        for (int k = 0; k < KC_STATE_DIM; k++)
        {
            P_chol[k][k] += dummy_value;
        }

        /* Compute inverse of P_chol */
        memcpy(tmp1, P_chol, sizeof(tmp1));
        mat_inv(&tmp1m, &Pc_inv_m);
        mat_mult(&Pc_inv_m, &x_errm, &e_x_m);

        /* Compute weight matrices */
        for (int state_k = 0; state_k < KC_STATE_DIM; state_k++)
        {
            GM_state(e_x[state_k], &wx_inv[state_k][state_k]);
            wx_inv[state_k][state_k] = 1.0f / wx_inv[state_k][state_k];
        }

        /* Rescale covariance matrix */
        mat_mult(&Pc_m, &wx_invm, &Pc_w_invm);
        mat_mult(&Pc_w_invm, &Pc_tran_m, &P_w_m);

        /* Rescale measurement variance */
        float w_y = 0.0f;
        float R_w = 0.0f;
        GM_UWB(e_y, &w_y);

        if (fabsf(w_y) < 0.0001f)
        {
            R_w = (R_chol * R_chol) / 0.0001f;
        }
        else
        {
            R_w = (R_chol * R_chol) / w_y;
        }

        /* Innovation covariance */
        mat_trans(&H, &HTm);
        mat_mult(&P_w_m, &HTm, &PHTm);

        float HPHR = R_w;
        for (int i = 0; i < KC_STATE_DIM; i++)
        {
            HPHR += h[i] * PHTd[i];
        }

        /* Calculate Kalman gain and update state */
        for (int i = 0; i < KC_STATE_DIM; i++)
        {
            Kw[i]      = PHTd[i] / HPHR;
            x_err[i]   = Kw[i] * error_check;
            X_state[i] = kf->S[i] + x_err[i];
        }

        /* Update P_iter and R_iter for next iteration */
        memcpy(P_iter, P_w, sizeof(P_iter));
        R_iter = R_w;
    }

    /* Final update using weighted matrices */
    kalmanCoreUpdateWithPKE(kf, &H, &Kwm, &P_w_m, error_check);
}

void kalmanCoreUpdateWithGravity(kalmanCoreData_t* kf, Axis3f* acc, float accelNoiseFactor)
{
    /* Normalize accelerometer measurement */
    float acc_norm = arm_sqrt(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);

    if (acc_norm < 0.1f * GRAVITY_MAGNITUDE || acc_norm > 3.0f * GRAVITY_MAGNITUDE)
    {
        /* Accelerometer reading is too far from gravity - likely accelerating */
        /* Skip this update to avoid corrupting attitude estimate */
        return;
    }

    float inv_norm          = 1.0f / acc_norm;
    float acc_normalized[3] = {acc->x * inv_norm, acc->y * inv_norm, acc->z * inv_norm};

    /* Expected gravity in body frame = R' * [0, 0, -g] */
    /* This gives us what the accelerometer should read if only gravity is present */
    float expected_acc[3] = {-kf->R[2][0] * GRAVITY_MAGNITUDE, -kf->R[2][1] * GRAVITY_MAGNITUDE,
                             -kf->R[2][2] * GRAVITY_MAGNITUDE};

    /* Normalize expected measurement */
    float expected_norm =
        arm_sqrt(expected_acc[0] * expected_acc[0] + expected_acc[1] * expected_acc[1] +
                 expected_acc[2] * expected_acc[2]);
    if (expected_norm < EPS)
    {
        return; /* Numerical issue */
    }

    float inv_exp_norm           = 1.0f / expected_norm;
    float expected_normalized[3] = {expected_acc[0] * inv_exp_norm, expected_acc[1] * inv_exp_norm,
                                    expected_acc[2] * inv_exp_norm};

    /* Measurement noise - scale based on how much we're accelerating */
    float deviation_from_gravity = fabsf(acc_norm - GRAVITY_MAGNITUDE) / GRAVITY_MAGNITUDE;
    float noise_std              = 0.1f * (1.0f + 5.0f * deviation_from_gravity) * accelNoiseFactor;

    /* Update with X component of normalized gravity */
    {
        float h[KC_STATE_DIM]     = {0};
        arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

        /* Jacobian: d(acc_x/|acc|) / d(attitude_error) */
        /* Approximation: treat as -R[2][0] affected by D1 and D2 */
        h[KC_STATE_D1] = kf->R[2][2];  /* Pitch affects X gravity */
        h[KC_STATE_D2] = -kf->R[2][1]; /* Yaw affects X gravity */

        float error = acc_normalized[0] - expected_normalized[0];
        kalmanCoreScalarUpdate(kf, &H, error, noise_std);
    }

    /* Update with Y component of normalized gravity */
    {
        float h[KC_STATE_DIM]     = {0};
        arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

        /* Jacobian: d(acc_y/|acc|) / d(attitude_error) */
        h[KC_STATE_D0] = -kf->R[2][2]; /* Roll affects Y gravity */
        h[KC_STATE_D2] = kf->R[2][0];  /* Yaw affects Y gravity */

        float error = acc_normalized[1] - expected_normalized[1];
        kalmanCoreScalarUpdate(kf, &H, error, noise_std);
    }
}

bool kalmanCoreFinalize(kalmanCoreData_t* kf)
{
    if (!kf->isUpdated)
    {
        return false;
    }

    /* Matrix to rotate attitude covariances */
    static float A[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float*)A};

    static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

    static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

    /* Get attitude error from state */
    float v0 = kf->S[KC_STATE_D0];
    float v1 = kf->S[KC_STATE_D1];
    float v2 = kf->S[KC_STATE_D2];

    /* Only incorporate if error is significant but bounded */
    if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) &&
        (fabsf(v0) < 10.0f && fabsf(v1) < 10.0f && fabsf(v2) < 10.0f))
    {
        /* Convert error to quaternion delta */
        float angle = arm_sqrt(v0 * v0 + v1 * v1 + v2 * v2) + EPS;
        float ca    = arm_cos_f32(angle / 2.0f);
        float sa    = arm_sin_f32(angle / 2.0f);
        float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

        /* Quaternion multiplication: q = dq * q */
        float tmpq0 = dq[0] * kf->q[0] - dq[1] * kf->q[1] - dq[2] * kf->q[2] - dq[3] * kf->q[3];
        float tmpq1 = dq[1] * kf->q[0] + dq[0] * kf->q[1] + dq[3] * kf->q[2] - dq[2] * kf->q[3];
        float tmpq2 = dq[2] * kf->q[0] - dq[3] * kf->q[1] + dq[0] * kf->q[2] + dq[1] * kf->q[3];
        float tmpq3 = dq[3] * kf->q[0] + dq[2] * kf->q[1] - dq[1] * kf->q[2] + dq[0] * kf->q[3];

        /* Normalize quaternion */
        float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPS;
        kf->q[0]   = tmpq0 / norm;
        kf->q[1]   = tmpq1 / norm;
        kf->q[2]   = tmpq2 / norm;
        kf->q[3]   = tmpq3 / norm;

        /* Rotate covariance matrix for attitude update */
        float d0 = v0 / 2.0f;
        float d1 = v1 / 2.0f;
        float d2 = v2 / 2.0f;

        memset(A, 0, sizeof(A));

        /* Position and velocity states unchanged */
        A[KC_STATE_X][KC_STATE_X]   = 1.0f;
        A[KC_STATE_Y][KC_STATE_Y]   = 1.0f;
        A[KC_STATE_Z][KC_STATE_Z]   = 1.0f;
        A[KC_STATE_PX][KC_STATE_PX] = 1.0f;
        A[KC_STATE_PY][KC_STATE_PY] = 1.0f;
        A[KC_STATE_PZ][KC_STATE_PZ] = 1.0f;

        /* Attitude covariance rotation */
        A[KC_STATE_D0][KC_STATE_D0] = 1.0f - d1 * d1 / 2.0f - d2 * d2 / 2.0f;
        A[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 / 2.0f;
        A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 / 2.0f;

        A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 / 2.0f;
        A[KC_STATE_D1][KC_STATE_D1] = 1.0f - d0 * d0 / 2.0f - d2 * d2 / 2.0f;
        A[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 / 2.0f;

        A[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 / 2.0f;
        A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 / 2.0f;
        A[KC_STATE_D2][KC_STATE_D2] = 1.0f - d0 * d0 / 2.0f - d1 * d1 / 2.0f;

        mat_trans(&Am, &tmpNN1m);              /* A' */
        mat_mult(&Am, &kf->Pm, &tmpNN2m);      /* A * P */
        mat_mult(&tmpNN2m, &tmpNN1m, &kf->Pm); /* A * P * A' */
    }

    /* Update rotation matrix from quaternion */
    float q0 = kf->q[0], q1 = kf->q[1], q2 = kf->q[2], q3 = kf->q[3];

    kf->R[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    kf->R[0][1] = 2.0f * (q1 * q2 - q0 * q3);
    kf->R[0][2] = 2.0f * (q1 * q3 + q0 * q2);

    kf->R[1][0] = 2.0f * (q1 * q2 + q0 * q3);
    kf->R[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    kf->R[1][2] = 2.0f * (q2 * q3 - q0 * q1);

    kf->R[2][0] = 2.0f * (q1 * q3 - q0 * q2);
    kf->R[2][1] = 2.0f * (q2 * q3 + q0 * q1);
    kf->R[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    /* Reset attitude error states */
    kf->S[KC_STATE_D0] = 0.0f;
    kf->S[KC_STATE_D1] = 0.0f;
    kf->S[KC_STATE_D2] = 0.0f;

    enforceSymmetryAndBounds(kf);
    kf->isUpdated = false;

    return true;
}

void kalmanCoreGetPosition(const kalmanCoreData_t* kf, float* x, float* y, float* z)
{
    *x = kf->S[KC_STATE_X];
    *y = kf->S[KC_STATE_Y];
    *z = kf->S[KC_STATE_Z];
}

void kalmanCoreGetVelocity(const kalmanCoreData_t* kf, float* vx, float* vy, float* vz)
{
    /* Velocity is in body frame, rotate to world frame */
    float px = kf->S[KC_STATE_PX];
    float py = kf->S[KC_STATE_PY];
    float pz = kf->S[KC_STATE_PZ];

    *vx = kf->R[0][0] * px + kf->R[0][1] * py + kf->R[0][2] * pz;
    *vy = kf->R[1][0] * px + kf->R[1][1] * py + kf->R[1][2] * pz;
    *vz = kf->R[2][0] * px + kf->R[2][1] * py + kf->R[2][2] * pz;
}

void kalmanCoreGetAttitude(const kalmanCoreData_t* kf, float* roll, float* pitch, float* yaw)
{
    float q0 = kf->q[0], q1 = kf->q[1], q2 = kf->q[2], q3 = kf->q[3];

    *yaw   = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
    *pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
    *roll  = atan2f(2.0f * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

void kalmanCoreGetQuaternion(const kalmanCoreData_t* kf, float* qw, float* qx, float* qy, float* qz)
{
    *qw = kf->q[0];
    *qx = kf->q[1];
    *qy = kf->q[2];
    *qz = kf->q[3];
}

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/

static void predictDt(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, Axis3f* acc,
                      Axis3f* gyro, float dt)
{
    /* Linearized dynamics matrix */
    static float A[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float*)A};

    static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

    static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

    float dt2 = dt * dt;

    /* ====== BUILD LINEARIZED DYNAMICS MATRIX ====== */
    memset(A, 0, sizeof(A));

    /* Initialize as identity */
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        A[i][i] = 1.0f;
    }

    /* Position from body-frame velocity: dX = R * v * dt */
    A[KC_STATE_X][KC_STATE_PX] = kf->R[0][0] * dt;
    A[KC_STATE_Y][KC_STATE_PX] = kf->R[1][0] * dt;
    A[KC_STATE_Z][KC_STATE_PX] = kf->R[2][0] * dt;

    A[KC_STATE_X][KC_STATE_PY] = kf->R[0][1] * dt;
    A[KC_STATE_Y][KC_STATE_PY] = kf->R[1][1] * dt;
    A[KC_STATE_Z][KC_STATE_PY] = kf->R[2][1] * dt;

    A[KC_STATE_X][KC_STATE_PZ] = kf->R[0][2] * dt;
    A[KC_STATE_Y][KC_STATE_PZ] = kf->R[1][2] * dt;
    A[KC_STATE_Z][KC_STATE_PZ] = kf->R[2][2] * dt;

    /* Position from attitude error */
    float py = kf->S[KC_STATE_PY];
    float pz = kf->S[KC_STATE_PZ];
    float px = kf->S[KC_STATE_PX];

    A[KC_STATE_X][KC_STATE_D0] = (py * kf->R[0][2] - pz * kf->R[0][1]) * dt;
    A[KC_STATE_Y][KC_STATE_D0] = (py * kf->R[1][2] - pz * kf->R[1][1]) * dt;
    A[KC_STATE_Z][KC_STATE_D0] = (py * kf->R[2][2] - pz * kf->R[2][1]) * dt;

    A[KC_STATE_X][KC_STATE_D1] = (-px * kf->R[0][2] + pz * kf->R[0][0]) * dt;
    A[KC_STATE_Y][KC_STATE_D1] = (-px * kf->R[1][2] + pz * kf->R[1][0]) * dt;
    A[KC_STATE_Z][KC_STATE_D1] = (-px * kf->R[2][2] + pz * kf->R[2][0]) * dt;

    A[KC_STATE_X][KC_STATE_D2] = (px * kf->R[0][1] - py * kf->R[0][0]) * dt;
    A[KC_STATE_Y][KC_STATE_D2] = (px * kf->R[1][1] - py * kf->R[1][0]) * dt;
    A[KC_STATE_Z][KC_STATE_D2] = (px * kf->R[2][1] - py * kf->R[2][0]) * dt;

    /* Body-frame velocity from body-frame velocity (gyro cross-product) */
    A[KC_STATE_PY][KC_STATE_PX] = -gyro->z * dt;
    A[KC_STATE_PZ][KC_STATE_PX] = gyro->y * dt;

    A[KC_STATE_PX][KC_STATE_PY] = gyro->z * dt;
    A[KC_STATE_PZ][KC_STATE_PY] = -gyro->x * dt;

    A[KC_STATE_PX][KC_STATE_PZ] = -gyro->y * dt;
    A[KC_STATE_PY][KC_STATE_PZ] = gyro->x * dt;

    /* Body-frame velocity from attitude error (gravity projection) */
    A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE * kf->R[2][2] * dt;
    A[KC_STATE_PZ][KC_STATE_D0] = GRAVITY_MAGNITUDE * kf->R[2][1] * dt;

    A[KC_STATE_PX][KC_STATE_D1] = GRAVITY_MAGNITUDE * kf->R[2][2] * dt;
    A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE * kf->R[2][0] * dt;

    A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE * kf->R[2][1] * dt;
    A[KC_STATE_PY][KC_STATE_D2] = GRAVITY_MAGNITUDE * kf->R[2][0] * dt;

    /* Attitude error from attitude error (second-order approximation) */
    float d0 = gyro->x * dt / 2.0f;
    float d1 = gyro->y * dt / 2.0f;
    float d2 = gyro->z * dt / 2.0f;

    A[KC_STATE_D0][KC_STATE_D0] = 1.0f - d1 * d1 / 2.0f - d2 * d2 / 2.0f;
    A[KC_STATE_D0][KC_STATE_D1] = d2 + d0 * d1 / 2.0f;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0 * d2 / 2.0f;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0 * d1 / 2.0f;
    A[KC_STATE_D1][KC_STATE_D1] = 1.0f - d0 * d0 / 2.0f - d2 * d2 / 2.0f;
    A[KC_STATE_D1][KC_STATE_D2] = d0 + d1 * d2 / 2.0f;

    A[KC_STATE_D2][KC_STATE_D0] = d1 + d0 * d2 / 2.0f;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1 * d2 / 2.0f;
    A[KC_STATE_D2][KC_STATE_D2] = 1.0f - d0 * d0 / 2.0f - d1 * d1 / 2.0f;

    /* ====== COVARIANCE PROPAGATION: P = A * P * A' ====== */
    mat_mult(&Am, &kf->Pm, &tmpNN1m);      /* A * P */
    mat_trans(&Am, &tmpNN2m);              /* A' */
    mat_mult(&tmpNN1m, &tmpNN2m, &kf->Pm); /* A * P * A' */

    /* ====== STATE PREDICTION ====== */
    /* For general motion (not just flight), we use full accelerometer data */

    /* Position update in body frame, then rotate to world frame */
    float dx_body = kf->S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
    float dy_body = kf->S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
    float dz_body = kf->S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f;

    kf->S[KC_STATE_X] += kf->R[0][0] * dx_body + kf->R[0][1] * dy_body + kf->R[0][2] * dz_body;
    kf->S[KC_STATE_Y] += kf->R[1][0] * dx_body + kf->R[1][1] * dy_body + kf->R[1][2] * dz_body;
    kf->S[KC_STATE_Z] += kf->R[2][0] * dx_body + kf->R[2][1] * dy_body + kf->R[2][2] * dz_body -
                         GRAVITY_MAGNITUDE * dt2 / 2.0f;

    /* Body-velocity update: accelerometers - gyro cross velocity - gravity in body frame */
    float tmpSPX = kf->S[KC_STATE_PX];
    float tmpSPY = kf->S[KC_STATE_PY];
    float tmpSPZ = kf->S[KC_STATE_PZ];

    kf->S[KC_STATE_PX] +=
        dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * kf->R[2][0]);
    kf->S[KC_STATE_PY] +=
        dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * kf->R[2][1]);
    kf->S[KC_STATE_PZ] +=
        dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * kf->R[2][2]);

    /* Attitude update via quaternion integration */
    float dtwx = dt * gyro->x;
    float dtwy = dt * gyro->y;
    float dtwz = dt * gyro->z;

    float angle = arm_sqrt(dtwx * dtwx + dtwy * dtwy + dtwz * dtwz) + EPS;
    float ca    = arm_cos_f32(angle / 2.0f);
    float sa    = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * dtwx / angle, sa * dtwy / angle, sa * dtwz / angle};

    /* Quaternion multiplication */
    float tmpq0 = dq[0] * kf->q[0] - dq[1] * kf->q[1] - dq[2] * kf->q[2] - dq[3] * kf->q[3];
    float tmpq1 = dq[1] * kf->q[0] + dq[0] * kf->q[1] + dq[3] * kf->q[2] - dq[2] * kf->q[3];
    float tmpq2 = dq[2] * kf->q[0] - dq[3] * kf->q[1] + dq[0] * kf->q[2] + dq[1] * kf->q[3];
    float tmpq3 = dq[3] * kf->q[0] + dq[2] * kf->q[1] - dq[1] * kf->q[2] + dq[0] * kf->q[3];

    /* Normalize quaternion */
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPS;
    kf->q[0]   = tmpq0 / norm;
    kf->q[1]   = tmpq1 / norm;
    kf->q[2]   = tmpq2 / norm;
    kf->q[3]   = tmpq3 / norm;

    kf->isUpdated = true;
}

static void addProcessNoiseDt(kalmanCoreData_t* kf, const kalmanCoreParams_t* params, float dt)
{
    /* Position process noise - CORRECTED: square each term then add */
    /* Variances add (not standard deviations), so we compute: */
    /* Q_pos = (sigma_acc * dt^2)^2 + (sigma_vel * dt)^2 + sigma_pos^2 */
    float acc_contrib_xy = params->procNoiseAcc_xy * dt * dt;
    float acc_contrib_z  = params->procNoiseAcc_z * dt * dt;
    float vel_contrib    = params->procNoiseVel * dt;

    float pos_noise_xy = acc_contrib_xy * acc_contrib_xy + vel_contrib * vel_contrib +
                         params->procNoisePos * params->procNoisePos;
    float pos_noise_z = acc_contrib_z * acc_contrib_z + vel_contrib * vel_contrib +
                        params->procNoisePos * params->procNoisePos;

    kf->P[KC_STATE_X][KC_STATE_X] += pos_noise_xy;
    kf->P[KC_STATE_Y][KC_STATE_Y] += pos_noise_xy;
    kf->P[KC_STATE_Z][KC_STATE_Z] += pos_noise_z;

    /* Velocity process noise - CORRECTED */
    float vel_noise_xy =
        acc_contrib_xy * acc_contrib_xy + params->procNoiseVel * params->procNoiseVel;
    float vel_noise_z = acc_contrib_z * acc_contrib_z + params->procNoiseVel * params->procNoiseVel;

    kf->P[KC_STATE_PX][KC_STATE_PX] += vel_noise_xy;
    kf->P[KC_STATE_PY][KC_STATE_PY] += vel_noise_xy;
    kf->P[KC_STATE_PZ][KC_STATE_PZ] += vel_noise_z;

    /* Attitude process noise - CORRECTED */
    float gyro_contrib_rp  = params->measNoiseGyro_rollpitch * dt;
    float gyro_contrib_yaw = params->measNoiseGyro_yaw * dt;
    float att_noise_rp =
        gyro_contrib_rp * gyro_contrib_rp + params->procNoiseAtt * params->procNoiseAtt;
    float att_noise_yaw =
        gyro_contrib_yaw * gyro_contrib_yaw + params->procNoiseAtt * params->procNoiseAtt;

    kf->P[KC_STATE_D0][KC_STATE_D0] += att_noise_rp;
    kf->P[KC_STATE_D1][KC_STATE_D1] += att_noise_rp;
    kf->P[KC_STATE_D2][KC_STATE_D2] += att_noise_yaw;

    enforceSymmetryAndBounds(kf);
}

static void enforceSymmetryAndBounds(kalmanCoreData_t* kf)
{
    for (int i = 0; i < KC_STATE_DIM; i++)
    {
        for (int j = i; j < KC_STATE_DIM; j++)
        {
            float p = 0.5f * kf->P[i][j] + 0.5f * kf->P[j][i];

            if (isnan(p) || p > MAX_COVARIANCE)
            {
                kf->P[i][j] = kf->P[j][i] = MAX_COVARIANCE;
            }
            else if (i == j && p < MIN_COVARIANCE)
            {
                kf->P[i][j] = kf->P[j][i] = MIN_COVARIANCE;
            }
            else
            {
                kf->P[i][j] = kf->P[j][i] = p;
            }
        }
    }
}
