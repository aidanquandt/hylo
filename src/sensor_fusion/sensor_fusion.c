/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "FreeRTOS.h"
#include "common.h"
#include "feature_config.h"
#include "system_halt.h"
#include "kalman_core.h"
#include "task_config.h"
#include "timer_driver.h"
#include "queue.h"
#include "task.h"
#include "wifi.h"
#include <math.h>
#include "sdcard_driver.h"
/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SENSOR_FUSION_IMU_QUEUE_SIZE 32U
#define SENSOR_FUSION_UWB_QUEUE_SIZE 16U
#define MIN_UPDATES_FOR_VALID 10
#define RANGING_DEFAULT_STDDEV_M 0.2f
#define MAX_VALID_POSITION_M 1000.0f
#define CONFIDENCE_RAMP_UPDATES 100
/** Run the full EKF predict step every N-th IMU event (200 Hz / 4 = 50 Hz). */
#define IMU_PREDICT_DECIMATION 4U

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void process_ranging_event(const sensor_event_t* event);
STATIC void process_imu_event(const sensor_event_t* event);
void sensor_fusion_init(void);
STATIC void sensor_fusion_task(void* pvParameters);
STATIC void sensor_fusion_10Hz_task(void* pvParameters);
STATIC void sensor_fusion_process_10Hz(void);
STATIC void sensor_fusion_update_position_estimate(void);

// Telemetry helpers (WIFI + SD card) — only used on HWREV 1
#if (HWREV == 1)
STATIC void send_ranging_telemetry(const sensor_ranging_data_t* ranging, uint32_t timestamp_ms);
STATIC void send_imu_telemetry(const sensor_event_t* event);
STATIC void send_position_telemetry(const sensor_fusion_position_t* position);
STATIC void sdcard_log_ranging(const sensor_ranging_data_t* ranging, uint32_t timestamp_ms);
STATIC void sdcard_log_imu(const sensor_event_t* event);
STATIC void sdcard_log_position(const sensor_fusion_position_t* position);
#endif


/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC QueueHandle_t sensor_queue                 = NULL; /* IMU events */
STATIC QueueHandle_t uwb_queue                    = NULL; /* UWB ranging events (higher priority) */
STATIC sensor_fusion_position_t position_estimate = {0};
STATIC bool fusion_initialized                    = false;
STATIC bool fusion_active                         = false;
STATIC bool debug_prints_enabled                  = false;
STATIC kalmanCoreData_t kf_data;
STATIC kalmanCoreParams_t kf_params;
STATIC uint32_t update_count                  = 0;
STATIC TaskHandle_t sensor_fusion_task_handle = NULL;
STATIC bool imu_enabled                       = false; // Disabled: IMU gravity compensation not yet validated

STATIC struct
{
    volatile uint32_t events_pushed;
    volatile uint32_t events_popped;
    volatile uint32_t overflows;
    volatile uint32_t max_depth;
    volatile uint32_t sequence;
} stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void process_ranging_event(const sensor_event_t* event)
{
    const sensor_ranging_data_t* ranging = &event->data.ranging;

    if (!ranging->anchor_position_valid)
    {
        if (debug_prints_enabled)
        {
        }
        return; // Cannot update without known anchor position
    }

    // When IMU is disabled there is no prediction step, so the covariance never grows.
    // Add process noise here so the Kalman gain stays non-trivial and position tracks movement.
    if (!imu_enabled)
    {
        kalmanCoreAddProcessNoise(&kf_data, &kf_params, event->timestamp_ms);
    }

    distanceMeasurement_t d = {.x        = ranging->anchor_position.x,
                               .y        = ranging->anchor_position.y,
                               .z        = ranging->anchor_position.z,
                               .distance = ranging->distance_m,
                               .stdDev   = RANGING_DEFAULT_STDDEV_M,
                               .anchorId = (uint8_t)(ranging->anchor_addr & 0xFF)};

    kalmanCoreUpdateWithDistance(&kf_data, &d);
    kalmanCoreFinalize(&kf_data);

    update_count++;

}

STATIC void process_imu_event(const sensor_event_t* event)
{
    static Axis3f imu_accum_acc  = {0};
    static Axis3f imu_accum_gyro = {0};
    static uint8_t imu_accum_count = 0;

    const sensor_imu_data_t* imu = &event->data.imu;

    /* Accumulate accel and gyro readings for averaging */
    imu_accum_acc.x  += imu->accel_x;
    imu_accum_acc.y  += imu->accel_y;
    imu_accum_acc.z  += imu->accel_z;
    imu_accum_gyro.x += imu->gyro_x;
    imu_accum_gyro.y += imu->gyro_y;
    imu_accum_gyro.z += imu->gyro_z;
    imu_accum_count++;

    if (imu_accum_count < IMU_PREDICT_DECIMATION)
    {
        return; /* Wait for more samples before running the expensive predict step */
    }

    /* Compute averaged IMU data over the decimation window */
    float inv_n = 1.0f / (float)imu_accum_count;
    Axis3f acc  = {.x = imu_accum_acc.x * inv_n,
                   .y = imu_accum_acc.y * inv_n,
                   .z = imu_accum_acc.z * inv_n};
    Axis3f gyro = {.x = imu_accum_gyro.x * inv_n,
                   .y = imu_accum_gyro.y * inv_n,
                   .z = imu_accum_gyro.z * inv_n};

    /* Reset accumulators */
    imu_accum_acc  = (Axis3f){0};
    imu_accum_gyro = (Axis3f){0};
    imu_accum_count = 0;

    /* Run full EKF predict at decimated rate (~50 Hz) */
    kalmanCorePredict(&kf_data, &kf_params, &acc, &gyro, event->timestamp_ms);
    kalmanCoreAddProcessNoise(&kf_data, &kf_params, event->timestamp_ms);

    /* Gravity constraint: correct roll/pitch from accelerometer when approximately stationary */
    float acc_mag_sq = acc.x * acc.x + acc.y * acc.y + acc.z * acc.z;
    float g_sq       = GRAVITY_MAGNITUDE * GRAVITY_MAGNITUDE;
    if (fabsf(acc_mag_sq - g_sq) < 1.0f * GRAVITY_MAGNITUDE)
    {
        kalmanCoreUpdateWithGravity(&kf_data, &acc, 1.0f);
    }

    kalmanCoreFinalize(&kf_data);
}

void sensor_fusion_init(void)
{
    sensor_queue = xQueueCreate(SENSOR_FUSION_IMU_QUEUE_SIZE, sizeof(sensor_event_t));
    if (sensor_queue == NULL)
    {
        system_halt("sensor_fusion", "Failed to create IMU queue");
    }

    uwb_queue = xQueueCreate(SENSOR_FUSION_UWB_QUEUE_SIZE, sizeof(sensor_event_t));
    if (uwb_queue == NULL)
    {
        system_halt("sensor_fusion", "Failed to create UWB queue");
    }

    stats.events_pushed = 0;
    stats.events_popped = 0;
    stats.overflows     = 0;
    stats.max_depth     = 0;
    stats.sequence      = 0;

    position_estimate.x            = 0.0f;
    position_estimate.y            = 0.0f;
    position_estimate.z            = 0.0f;
    position_estimate.vx           = 0.0f;
    position_estimate.vy           = 0.0f;
    position_estimate.vz           = 0.0f;
    position_estimate.confidence   = 0.0f;
    position_estimate.timestamp_ms = 0;
    position_estimate.valid        = false;

    kalmanCoreDefaultParams(&kf_params);
    kalmanCoreInit(&kf_data, &kf_params, timer_driver_get_time_ms());

    update_count       = 0;
    fusion_initialized = true;

    BaseType_t result = xTaskCreate(sensor_fusion_task, "sensor_fusion", TASK_STACK_8KB,
                                    NULL, TASK_PRIORITY_SENSOR_FUSION, &sensor_fusion_task_handle);

    if (result != pdPASS)
    {
        system_halt("sensor_fusion", "Failed to create processing task");
    }

    result = xTaskCreate(sensor_fusion_10Hz_task, "sf_10hz", TASK_STACK_2KB,
                         NULL, TASK_PRIORITY_SENSOR_FUSION, NULL);
    if (result != pdPASS)
    {
        system_halt("sensor_fusion", "Failed to create 10Hz task");
    }
}

STATIC void sensor_fusion_10Hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        sensor_fusion_process_10Hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));
    }
}

STATIC void sensor_fusion_task(void* pvParameters)
{
    (void)pvParameters;
    sensor_event_t event;
    bool got_event;

    while (1)
    {
        got_event = false;

        /* ---- UWB first: drain all pending ranging events before touching IMU ---- */
        while (xQueueReceive(uwb_queue, &event, 0) == pdPASS)
        {
            got_event = true;
            stats.events_popped++;

            #if (HWREV == 1)
            send_ranging_telemetry(&event.data.ranging, event.timestamp_ms);
            sdcard_log_ranging(&event.data.ranging, event.timestamp_ms);
            #endif

            if (fusion_active)
            {
                process_ranging_event(&event);
                sensor_fusion_update_position_estimate();
            }
        }

        /* ---- Then process one IMU event (block briefly if nothing available) ----
         * Use a short timeout (1 ms) so we loop back to check the UWB queue quickly
         * if a ranging event arrives while we are waiting for IMU data. */
        if (xQueueReceive(sensor_queue, &event, got_event ? 0 : pdMS_TO_TICKS(1)) == pdPASS)
        {
            stats.events_popped++;

            #if (HWREV == 1)
            send_imu_telemetry(&event);
            sdcard_log_imu(&event);
            #endif

            if (fusion_active && imu_enabled)
            {
                process_imu_event(&event);
                sensor_fusion_update_position_estimate();
            }
        }
    }
}

/**
 * @brief Update the position estimate from Kalman filter state
 */
STATIC void sensor_fusion_process_10Hz(void)
{
#if FEATURE_PRINT_SENSOR_FUSION_LOCATION_ESTIMATE
    if (!fusion_initialized)
    {
        return;
    }

    sensor_fusion_position_t pos;
    if (debug_prints_enabled)
    {
        if (sensor_fusion_get_position(&pos))
        {
            (void)pos;
        }
        else if (fusion_active)
        {
        }
    }
#endif
}

STATIC void sensor_fusion_update_position_estimate(void)
{
    // Extract position from Kalman filter
    kalmanCoreGetPosition(&kf_data, &position_estimate.x, &position_estimate.y,
                          &position_estimate.z);

    // Extract velocity from Kalman filter
    kalmanCoreGetVelocity(&kf_data, &position_estimate.vx, &position_estimate.vy,
                          &position_estimate.vz);

    // Update timestamp
    position_estimate.timestamp_ms = timer_driver_get_time_ms();

    // Check validity: need enough updates and reasonable position
    bool valid_updates  = (update_count >= MIN_UPDATES_FOR_VALID);
    bool reasonable_pos = (fabsf(position_estimate.x) < MAX_VALID_POSITION_M &&
                           fabsf(position_estimate.y) < MAX_VALID_POSITION_M &&
                           fabsf(position_estimate.z) < MAX_VALID_POSITION_M);
    bool no_nans =
        (!isnan(position_estimate.x) && !isnan(position_estimate.y) && !isnan(position_estimate.z) &&
         !isnan(position_estimate.vx) && !isnan(position_estimate.vy) && !isnan(position_estimate.vz));

    if (!no_nans || !reasonable_pos)
    {
        // Filter has diverged — reset to recover
        kalmanCoreInit(&kf_data, &kf_params, timer_driver_get_time_ms());
        update_count = 0;
    }

    position_estimate.valid = valid_updates && reasonable_pos && no_nans;

    (void)no_nans;
    (void)reasonable_pos;

    // Simple confidence metric based on update count
    position_estimate.confidence = (update_count < CONFIDENCE_RAMP_UPDATES)
                                       ? (float)update_count / (float)CONFIDENCE_RAMP_UPDATES
                                       : 1.0f;
    
    position_estimate.imu_enable = imu_enabled;

    #if (HWREV == 1)
        // Send position estimate to WiFi telemetry
        send_position_telemetry(&position_estimate);
        // Log position estimate to SD card
        sdcard_log_position(&position_estimate);
    #endif
}

/*---------------------------------------------------------------------------
 * Telemetry Helper Functions (HWREV 1 only: WiFi + SD card)
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)

/**
 * @brief Send ranging telemetry to WiFi (non-blocking)
 */
STATIC void send_ranging_telemetry(const sensor_ranging_data_t* ranging, uint32_t timestamp_ms)
{
    (void)ranging;
    (void)timestamp_ms;
}

/**
 * @brief Send IMU telemetry to WiFi (decimated to 10Hz)
 */
STATIC void send_imu_telemetry(const sensor_event_t* event)
{
    (void)event;
}

/**
 * @brief Send position estimate telemetry to WiFi (throttled to 10Hz)
 */
STATIC void send_position_telemetry(const sensor_fusion_position_t* position)
{
    (void)position;
}

/**
 * @brief Log ranging event to SD card (non-blocking)
 */
STATIC void sdcard_log_ranging(const sensor_ranging_data_t* ranging, uint32_t timestamp_ms)
{
    sdcard_driver_event_t entry = {
        .type         = SDCARD_DRIVER_EVENT_RANGING,
        .timestamp_ms = timestamp_ms,
        .data.ranging = {
            .distance_m = ranging->distance_m,
            .anchor_addr = ranging->anchor_addr,
            .anchor_x = ranging->anchor_position.x,
            .anchor_y = ranging->anchor_position.y,
            .anchor_z = ranging->anchor_position.z,
            .quality = ranging->quality,
            .rssi_dbm = ranging->rssi_dbm,
        },
    };
    (void)sdcard_driver_push_event(&entry);
}

/**
 * @brief Log every IMU event to SD card
 */
STATIC void sdcard_log_imu(const sensor_event_t* event)
{
    sdcard_driver_event_t entry = {
        .type         = SDCARD_DRIVER_EVENT_IMU,
        .timestamp_ms = event->timestamp_ms,
        .data.imu = {
            .accel_x = event->data.imu.accel_x,
            .accel_y = event->data.imu.accel_y,
            .accel_z = event->data.imu.accel_z,
            .gyro_x = event->data.imu.gyro_x,
            .gyro_y = event->data.imu.gyro_y,
            .gyro_z = event->data.imu.gyro_z,
            .temp_c = event->data.imu.temp_c,
        },
    };
    (void)sdcard_driver_push_event(&entry);
}

/**
 * @brief Log position estimate to SD card (throttled to 10Hz, valid only)
 */
STATIC void sdcard_log_position(const sensor_fusion_position_t* position)
{
    static uint32_t last_sd_position_send = 0;
    uint32_t now = timer_driver_get_time_ms();

    if ((now - last_sd_position_send) >= 100U) // Throttle to max 10 Hz CHANGE IF WANT MORE FREQUENT LOGGING!!!
    {
        if (position->valid)
        {
            sdcard_driver_event_t entry = {
                .type             = SDCARD_DRIVER_EVENT_POSITION,
                .timestamp_ms     = position->timestamp_ms,
                .data.position = {
                    .x = position->x,
                    .y = position->y,
                    .z = position->z,
                    .vx = position->vx,
                    .vy = position->vy,
                    .vz = position->vz,
                    .confidence = position->confidence,
                    .valid = position->valid,
                    .imu_enable = imu_enabled,
                },
            };
            (void)sdcard_driver_push_event(&entry);
            last_sd_position_send = now;
        }
    }
}

#endif /* HWREV == 1 */


/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

sensor_fusion_status_e sensor_fusion_push_event(const sensor_event_t* event)
{
    if (event == NULL)
    {
        return SENSOR_FUSION_ERROR_NULL_PTR;
    }

    if (!fusion_initialized)
    {
        return SENSOR_FUSION_ERROR_NOT_INITIALIZED;
    }

    if (event->type >= NUM_SENSOR_EVENT_TYPES)
    {
        return SENSOR_FUSION_ERROR_INVALID_TYPE;
    }

    sensor_event_t queued_event = *event;

    taskENTER_CRITICAL();
    queued_event.sequence = stats.sequence++;
    taskEXIT_CRITICAL();

    /* Route ranging events to dedicated UWB queue so they are never starved by IMU traffic */
    QueueHandle_t target_queue =
        (event->type == SENSOR_EVENT_RANGING) ? uwb_queue : sensor_queue;

    BaseType_t result = xQueueSend(target_queue, &queued_event, 0);

    if (result == pdPASS)
    {
        stats.events_pushed++;

        UBaseType_t depth = uxQueueMessagesWaiting(target_queue);
        if (depth > stats.max_depth)
        {
            stats.max_depth = depth;
        }

        return SENSOR_FUSION_SUCCESS;
    }

    /* Queue full - drop oldest event and add new one.
     * This prevents blocking the caller and ensures recent data is processed. */
    sensor_event_t discarded;
    if (xQueueReceive(target_queue, &discarded, 0) == pdPASS &&
        xQueueSend(target_queue, &queued_event, 0) == pdPASS)
    {
        stats.overflows++;
        stats.events_pushed++;
        return SENSOR_FUSION_SUCCESS;
    }

    stats.overflows++;
    return SENSOR_FUSION_ERROR_QUEUE_FULL;
}

sensor_fusion_status_e sensor_fusion_pop_event(sensor_event_t* event)
{
    if (event == NULL)
    {
        return SENSOR_FUSION_ERROR_NULL_PTR;
    }

    if (!fusion_initialized)
    {
        return SENSOR_FUSION_ERROR_NOT_INITIALIZED;
    }

    // Non-blocking receive for external/debug use only
    // Normal processing uses blocking receive in dedicated task
    if (xQueueReceive(sensor_queue, event, 0) == pdPASS)
    {
        stats.events_popped++;
        return SENSOR_FUSION_SUCCESS;
    }

    return SENSOR_FUSION_ERROR_QUEUE_EMPTY;
}

void sensor_fusion_get_stats(sensor_queue_stats_t* stats_out)
{
    if (stats_out == NULL || !fusion_initialized)
    {
        return;
    }

    // Read statistics (atomic reads on Cortex-M are safe for uint32_t)
    stats_out->events_pushed     = stats.events_pushed;
    stats_out->events_popped     = stats.events_popped;
    stats_out->overflows         = stats.overflows;
    stats_out->current_depth     = (uint32_t)uxQueueMessagesWaiting(sensor_queue);
    stats_out->max_depth_reached = stats.max_depth;
}

bool sensor_fusion_get_position(sensor_fusion_position_t* position)
{
    if (position == NULL || !fusion_initialized)
    {
        return false;
    }

    // Use FreeRTOS critical section (lighter than disabling all interrupts)
    taskENTER_CRITICAL();
    *position  = position_estimate;
    bool valid = position_estimate.valid;
    taskEXIT_CRITICAL();

    return valid;
}

void sensor_fusion_reset(void)
{
    if (!fusion_initialized)
    {
        return;
    }

    // Clear queue
    xQueueReset(sensor_queue);

    // Reset Kalman filter to initial state
    kalmanCoreInit(&kf_data, &kf_params, timer_driver_get_time_ms());

    // Reset position estimate validity
    taskENTER_CRITICAL();
    position_estimate.valid      = false;
    position_estimate.confidence = 0.0f;
    taskEXIT_CRITICAL();

    // Reset update counter
    update_count = 0;
}

void sensor_fusion_get_position_xyz(float* x, float* y, float* z)
{
    if (x == NULL || y == NULL || z == NULL || !fusion_initialized)
    {
        if (x)
            *x = 0.0f;
        if (y)
            *y = 0.0f;
        if (z)
            *z = 0.0f;
        return;
    }

    taskENTER_CRITICAL();
    *x = position_estimate.x;
    *y = position_estimate.y;
    *z = position_estimate.z;
    taskEXIT_CRITICAL();
}

void sensor_fusion_get_velocity(float* vx, float* vy, float* vz)
{
    if (vx == NULL || vy == NULL || vz == NULL || !fusion_initialized)
    {
        if (vx)
            *vx = 0.0f;
        if (vy)
            *vy = 0.0f;
        if (vz)
            *vz = 0.0f;
        return;
    }

    taskENTER_CRITICAL();
    *vx = position_estimate.vx;
    *vy = position_estimate.vy;
    *vz = position_estimate.vz;
    taskEXIT_CRITICAL();
}

void sensor_fusion_get_attitude(float* roll, float* pitch, float* yaw)
{
    if (roll == NULL || pitch == NULL || yaw == NULL || !fusion_initialized)
    {
        if (roll)
            *roll = 0.0f;
        if (pitch)
            *pitch = 0.0f;
        if (yaw)
            *yaw = 0.0f;
        return;
    }

    // Get attitude directly from Kalman filter
    kalmanCoreGetAttitude(&kf_data, roll, pitch, yaw);
}

bool sensor_fusion_is_valid(void)
{
    if (!fusion_initialized)
    {
        return false;
    }

    taskENTER_CRITICAL();
    bool valid = position_estimate.valid;
    taskEXIT_CRITICAL();

    return valid;
}

void sensor_fusion_start(void)
{
    if (!fusion_initialized)
    {
        return;
    }

    // Reset/clear all data when starting
    sensor_fusion_reset();

    // Activate sensor fusion
    fusion_active = true;
}

void sensor_fusion_stop(void)
{
    if (!fusion_initialized)
    {
        return;
    }

    // Deactivate sensor fusion (events will be discarded)
    fusion_active = false;
}

bool sensor_fusion_is_active(void)
{
    return fusion_active;
}

void sensor_fusion_enable_debug_prints(bool enable)
{
    debug_prints_enabled = enable;
}

bool sensor_fusion_get_debug_prints_enabled(void)
{
    return debug_prints_enabled;
}

void sensor_fusion_enable_imu(bool enable)
{
    imu_enabled = enable;
}

bool sensor_fusion_get_imu_enabled(void)
{
    return imu_enabled;
}

void sensor_fusion_set_process_noise(float pos, float vel, float att)
{
    if (!fusion_initialized)
    {
        return;
    }

    kf_params.procNoisePos = pos;
    kf_params.procNoiseVel = vel;
    kf_params.procNoiseAtt = att;
}

void sensor_fusion_get_process_noise(float* pos, float* vel, float* att)
{
    if (!fusion_initialized || pos == NULL || vel == NULL || att == NULL)
    {
        return;
    }

    *pos = kf_params.procNoisePos;
    *vel = kf_params.procNoiseVel;
    *att = kf_params.procNoiseAtt;
}

void sensor_fusion_get_kalman_params(kalmanCoreParams_t* params)
{
    if (!fusion_initialized || params == NULL)
    {
        return;
    }

    *params = kf_params;
}

void sensor_fusion_set_kalman_params(const kalmanCoreParams_t* params)
{
    if (!fusion_initialized || params == NULL)
    {
        return;
    }

    kf_params = *params;
}