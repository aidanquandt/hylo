/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "FreeRTOS.h"
#include "counter.h"
#include "error_handler.h"
#include "feature_config.h"
#include "module.h"
#include "queue.h"
#include "uart_manager.h"
#include "common.h"
#include "kalman/kalman_core.h"
#include "outlier_rejection.h"
#include <math.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SENSOR_FUSION_QUEUE_SIZE 32U
#define MIN_UPDATES_FOR_VALID 10
#define RANGING_DEFAULT_STDDEV_M 0.15f
#define MAX_VALID_POSITION_M 1000.0f
#define CONFIDENCE_RAMP_UPDATES 100
#define SENSOR_FUSION_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define SENSOR_FUSION_STACK_SIZE 2048

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void process_ranging_event(const sensor_event_t* event);
STATIC void process_imu_event(const sensor_event_t* event);
STATIC void sensor_fusion_init(void);
STATIC void sensor_fusion_task(void* pvParameters);
STATIC void sensor_fusion_update_position_estimate(void);
STATIC void sensor_fusion_process_10Hz(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void sensor_fusion_init(void);
extern const module_S sensor_fusion_module;

const module_S sensor_fusion_module = {
    .module_name         = "sensor_fusion",
    .module_init         = sensor_fusion_init,
    .module_process_10Hz = sensor_fusion_process_10Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC QueueHandle_t sensor_queue                 = NULL;
STATIC sensor_fusion_position_t position_estimate = {0};
STATIC bool fusion_initialized                    = false;
STATIC bool fusion_active                         = false;
STATIC bool debug_prints_enabled                  = false;
STATIC kalmanCoreData_t kf_data;
STATIC kalmanCoreParams_t kf_params;
STATIC uint32_t update_count = 0;
STATIC TaskHandle_t sensor_fusion_task_handle = NULL;
STATIC bool imu_enabled = true;  // Enable IMU by default

STATIC struct
{
    volatile uint32_t events_pushed;
    volatile uint32_t events_popped;
    volatile uint32_t overflows;
    volatile uint32_t max_depth;
    volatile uint32_t sequence;
    volatile uint32_t ranging_rejected;     // UWB measurements rejected as outliers
    volatile uint32_t ranging_accepted;     // UWB measurements accepted
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
            uart_manager_print("SF_DEBUG: Ranging skipped - anchor position invalid\r\n");
        }
        return;  // Cannot update without known anchor position
    }
    
    distanceMeasurement_t d = {
        .x = ranging->anchor_position.x,
        .y = ranging->anchor_position.y,
        .z = ranging->anchor_position.z,
        .distance = ranging->distance_m,
        .stdDev = RANGING_DEFAULT_STDDEV_M,
        .anchorId = (uint8_t)(ranging->anchor_addr & 0xFF)
    };
    
    // Outlier rejection using innovation-based gating
    float innovation, innovation_variance;
    bool is_valid = outlier_validate_ranging(&kf_data, &d, &innovation, &innovation_variance);
    
    if (!is_valid)
    {
        stats.ranging_rejected++;
        if (debug_prints_enabled)
        {
            float mahal = outlier_mahalanobis_distance(innovation, innovation_variance);
            uart_manager_print("SF_OUTLIER: Rejected ranging from 0x%04X: "
                             "innov=%.3f, var=%.3f, mahal=%.2f\r\n",
                             ranging->anchor_addr, innovation, innovation_variance, mahal);
        }
        return;
    }
    
    // Adaptive measurement noise based on innovation consistency
    d.stdDev = outlier_adaptive_measurement_noise(RANGING_DEFAULT_STDDEV_M, 
                                                   innovation, 
                                                   innovation_variance);
    
    stats.ranging_accepted++;
    
    // kalmanCoreAddProcessNoise(&kf_data, &kf_params, event->timestamp_ms);
    kalmanCoreUpdateWithDistance(&kf_data, &d);
    kalmanCoreFinalize(&kf_data);
    
    update_count++;

    // DEBUG: Log every 10th update
    if (debug_prints_enabled && (update_count % 10 == 0))
    {
        uart_manager_print("SF_DEBUG: Ranging stats - accepted:%lu rejected:%lu (%.1f%% reject)\r\n",
                          (unsigned long)stats.ranging_accepted, (unsigned long)stats.ranging_rejected,
                          100.0f * stats.ranging_rejected / (stats.ranging_accepted + stats.ranging_rejected + 1));
    }
}

STATIC void process_imu_event(const sensor_event_t* event)
{
    const sensor_imu_data_t* imu = &event->data.imu;

    Axis3f acc = {
        .x = imu->accel_x,
        .y = imu->accel_y,
        .z = imu->accel_z
    };
    
    Axis3f gyro = {
        .x = imu->gyro_x,
        .y = imu->gyro_y,
        .z = imu->gyro_z
    };
    
    /* Predict state using IMU measurements */
    kalmanCorePredict(&kf_data, &kf_params, &acc, &gyro, event->timestamp_ms);
    kalmanCoreAddProcessNoise(&kf_data, &kf_params, event->timestamp_ms);
    
    /* DISABLED: Gravity constraint - causing divergence issues */
    /* TODO: Debug and re-enable after fixing Jacobian */
    // kalmanCoreUpdateWithGravity(&kf_data, &acc, 1.0f);
    
    kalmanCoreFinalize(&kf_data);
}

STATIC void sensor_fusion_init(void)
{
    sensor_queue = xQueueCreate(SENSOR_FUSION_QUEUE_SIZE, sizeof(sensor_event_t));
    if (sensor_queue == NULL)
    {
        error_handler_fatal("sensor_fusion", "Failed to create event queue");
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
    kalmanCoreInit(&kf_data, &kf_params, xTaskGetTickCount());
    
    update_count = 0;
    fusion_initialized = true;
    
    BaseType_t result = xTaskCreate(
        sensor_fusion_task,
        "sensor_fusion",
        SENSOR_FUSION_STACK_SIZE,
        NULL,
        SENSOR_FUSION_TASK_PRIORITY,
        &sensor_fusion_task_handle
    );
    
    if (result != pdPASS)
    {
        error_handler_fatal("sensor_fusion", "Failed to create processing task");
    }
}

STATIC void sensor_fusion_task(void* pvParameters)
{
    (void)pvParameters;
    sensor_event_t event;
    
    while (1)
    {
        // Block indefinitely waiting for sensor event (no timeout)
        // Task sleeps here consuming zero CPU until event arrives
        if (xQueueReceive(sensor_queue, &event, portMAX_DELAY) == pdPASS)
        {
            stats.events_popped++;
            
            // Discard events when sensor fusion is not active
            if (!fusion_active)
            {
                continue;
            }
            
            switch (event.type)
            {
                case SENSOR_EVENT_RANGING:
                    process_ranging_event(&event);
                    break;

                case SENSOR_EVENT_IMU:
                    if (imu_enabled)
                    {
                        process_imu_event(&event);
                    }
                    break;

                default:
                    // Unknown event type - should never happen due to validation
                    break;
            }
            
            // Update position estimate after each event
            sensor_fusion_update_position_estimate();
        }
    }
}

/**
 * @brief Update the position estimate from Kalman filter state
 */
STATIC void sensor_fusion_update_position_estimate(void)
{
    // Extract position from Kalman filter
    kalmanCoreGetPosition(&kf_data, 
                         &position_estimate.x, 
                         &position_estimate.y, 
                         &position_estimate.z);
    
    // Extract velocity from Kalman filter
    kalmanCoreGetVelocity(&kf_data, 
                         &position_estimate.vx, 
                         &position_estimate.vy, 
                         &position_estimate.vz);
    
    // Update timestamp
    position_estimate.timestamp_ms = xTaskGetTickCount();
    
    // Check validity: need enough updates and reasonable position
    bool valid_updates = (update_count >= MIN_UPDATES_FOR_VALID);
    bool reasonable_pos = (fabsf(position_estimate.x) < MAX_VALID_POSITION_M &&
                          fabsf(position_estimate.y) < MAX_VALID_POSITION_M &&
                          fabsf(position_estimate.z) < MAX_VALID_POSITION_M);
    bool no_nans = (!isnan(position_estimate.x) && 
                   !isnan(position_estimate.y) && 
                   !isnan(position_estimate.z));
    
    position_estimate.valid = valid_updates && reasonable_pos && no_nans;

    // DEBUG: Log why position is invalid
    if (debug_prints_enabled && !position_estimate.valid)
    {
        uart_manager_print("SF_DEBUG: valid=%d, updates=%lu (need %d), pos=(%.2f,%.2f,%.2f), noNaN=%d, reasonable=%d\r\n",
                          position_estimate.valid,
                          (unsigned long)update_count,
                          MIN_UPDATES_FOR_VALID,
                          position_estimate.x,
                          position_estimate.y,
                          position_estimate.z,
                          no_nans,
                          reasonable_pos);
    }
    
    // Simple confidence metric based on update count
    position_estimate.confidence = (update_count < CONFIDENCE_RAMP_UPDATES) ? 
                                   (float)update_count / (float)CONFIDENCE_RAMP_UPDATES : 1.0f;
}

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

    BaseType_t result = xQueueSend(sensor_queue, &queued_event, 0);

    if (result == pdPASS)
    {
        stats.events_pushed++;

        UBaseType_t depth = uxQueueMessagesWaiting(sensor_queue);
        if (depth > stats.max_depth)
        {
            stats.max_depth = depth;
        }

        return SENSOR_FUSION_SUCCESS;
    }

    // Queue full - drop oldest event and add new one
    // This prevents blocking the caller and ensures recent data is processed
    sensor_event_t discarded;
    if (xQueueReceive(sensor_queue, &discarded, 0) == pdPASS &&
        xQueueSend(sensor_queue, &queued_event, 0) == pdPASS)
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
    kalmanCoreInit(&kf_data, &kf_params, xTaskGetTickCount());
    
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
        if (x) *x = 0.0f;
        if (y) *y = 0.0f;
        if (z) *z = 0.0f;
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
        if (vx) *vx = 0.0f;
        if (vy) *vy = 0.0f;
        if (vz) *vz = 0.0f;
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
        if (roll) *roll = 0.0f;
        if (pitch) *pitch = 0.0f;
        if (yaw) *yaw = 0.0f;
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
    uart_manager_print("SF: IMU %s\r\n", enable ? "enabled" : "disabled");
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
    
    uart_manager_print("SF: Process noise updated - pos=%.4f, vel=%.4f, att=%.4f\r\n",
                      pos, vel, att);
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
    uart_manager_print("SF: Kalman parameters updated\r\n");
}

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
            uart_manager_print("SF: x=%6.2f y=%6.2f z=%6.2f m | vx=%6.2f vy=%6.2f vz=%6.2f m/s | conf=%.2f\r\n",
                            pos.x, pos.y, pos.z,
                            pos.vx, pos.vy, pos.vz,
                            pos.confidence);
        }
        else if (fusion_active)
        {
            uart_manager_print("SF: Position estimate invalid\r\n");
        }
    }
#endif
}