/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "FreeRTOS.h"
#include "counter.h"
#include "error_handler.h"
#include "module.h"
#include "queue.h"
#include "sensor_fusion_types.h"
#include "common.h"

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void process_ranging_event(const sensor_event_t* event);
STATIC void process_imu_event(const sensor_event_t* event);
STATIC void sensor_fusion_init(void);
STATIC void sensor_fusion_process_input_queue(void);
STATIC void sensor_fusion_algorithm(void);
STATIC void sensor_fusion_process_200Hz(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void sensor_fusion_init(void);
STATIC void sensor_fusion_process_1kHz(void);
extern const module_S sensor_fusion_module;

const module_S sensor_fusion_module = {
    .module_name         = "sensor_fusion",
    .module_init         = sensor_fusion_init,
    .module_process_1kHz = sensor_fusion_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC QueueHandle_t sensor_queue                 = NULL;
STATIC sensor_fusion_position_t position_estimate = {0};
STATIC bool fusion_initialized                    = false;

// Statistics (still tracked manually for overflow detection)
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
    // TODO: Implement ranging processing
    // For now, just a placeholder
    (void)event;
}

STATIC void process_imu_event(const sensor_event_t* event)
{
    // TODO: Implement IMU processing
    // For now, just a placeholder
    (void)event;
}

STATIC void sensor_fusion_init(void)
{
    // Create FreeRTOS queue (called before scheduler starts)
    sensor_queue = xQueueCreate(SENSOR_FUSION_QUEUE_SIZE, sizeof(sensor_event_t));
    if (sensor_queue == NULL)
    {
        error_handler_fatal("sensor_fusion", "Failed to create event queue");
    }

    // Initialize statistics
    stats.events_pushed = 0;
    stats.events_popped = 0;
    stats.overflows     = 0;
    stats.max_depth     = 0;
    stats.sequence      = 0;

    // Initialize position estimate
    position_estimate.x            = 0.0f;
    position_estimate.y            = 0.0f;
    position_estimate.z            = 0.0f;
    position_estimate.vx           = 0.0f;
    position_estimate.vy           = 0.0f;
    position_estimate.vz           = 0.0f;
    position_estimate.confidence   = 0.0f;
    position_estimate.timestamp_ms = 0;
    position_estimate.valid        = false;

    fusion_initialized = true;
}

STATIC void sensor_fusion_process_input_queue(void)
{
    if (!fusion_initialized)
    {
        return;
    }

    // Process all queued events
    sensor_event_t event;
    while (sensor_fusion_pop_event(&event) == SENSOR_FUSION_SUCCESS)
    {
        // Dispatch based on event type
        switch (event.type)
        {
            case SENSOR_EVENT_RANGING:
                process_ranging_event(&event);
                break;

            case SENSOR_EVENT_IMU:
                process_imu_event(&event);
                break;

            case SENSOR_EVENT_MAGNETOMETER:
                // TODO: Implement magnetometer processing
                break;

            default:
                // Unknown event type - should never happen due to validation
                break;
        }
    }
}

STATIC void sensor_fusion_algorithm(void)
{
    // TODO: Run sensor fusion algorithm here
    // This is where you would implement Kalman filter, complementary filter, etc.
    // For now, just update timestamp
    position_estimate.timestamp_ms = xTaskGetTickCount();
}

STATIC void sensor_fusion_process_200Hz(void)
{
    sensor_fusion_process_input_queue();
    sensor_fusion_algorithm();
}

STATIC void sensor_fusion_process_1kHz(void)
{
    // Prescaler: Run at 200Hz (every 5th call of 1kHz)
    STATIC uint8_t prescaler_counter = 0;
    if (counter_uint8_t(&prescaler_counter, 5))
    {
        sensor_fusion_process_200Hz();
    }
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
        return SENSOR_FUSION_ERROR_NULL_PTR; // Module not initialized
    }

    // Validate event type
    if (event->type >= SENSOR_EVENT_MAGNETOMETER + 1)
    {
        return SENSOR_FUSION_ERROR_INVALID_TYPE;
    }

    // Copy event and assign sequence number
    sensor_event_t queued_event = *event;

    // Atomically increment sequence (safe for task context)
    taskENTER_CRITICAL();
    queued_event.sequence = stats.sequence++;
    taskEXIT_CRITICAL();

    // Try to push to queue (non-blocking, called from task context)
    BaseType_t result = xQueueSend(sensor_queue, &queued_event, 0);

    if (result == pdPASS)
    {
        // Success
        stats.events_pushed++;

        // Track max depth
        UBaseType_t depth = uxQueueMessagesWaiting(sensor_queue);
        if (depth > stats.max_depth)
        {
            stats.max_depth = depth;
        }

        return SENSOR_FUSION_SUCCESS;
    }
    else
    {
        // Queue full - implement drop-oldest policy manually
        // Receive (and discard) oldest event, then try again
        sensor_event_t discarded;
        if (xQueueReceive(sensor_queue, &discarded, 0) == pdPASS)
        {
            stats.overflows++;

            // Now try to add new event
            result = xQueueSend(sensor_queue, &queued_event, 0);
            if (result == pdPASS)
            {
                stats.events_pushed++;
                return SENSOR_FUSION_SUCCESS;
            }
        }

        // Failed to recover from full queue
        stats.overflows++;
        return SENSOR_FUSION_ERROR_QUEUE_FULL;
    }
}

sensor_fusion_status_e sensor_fusion_pop_event(sensor_event_t* event)
{
    if (event == NULL)
    {
        return SENSOR_FUSION_ERROR_NULL_PTR;
    }

    if (!fusion_initialized)
    {
        return SENSOR_FUSION_ERROR_NULL_PTR; // Module not initialized
    }

    // Non-blocking receive (called from 1kHz task context)
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

    // Reset position estimate validity
    taskENTER_CRITICAL();
    position_estimate.valid      = false;
    position_estimate.confidence = 0.0f;
    taskEXIT_CRITICAL();
}