/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb.h"
#include "FreeRTOS.h"
#include "error_handler.h"
#include "mac_802154.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_timer.h"
#include "queue.h"
#include "semphr.h"
#include "state_machine.h"
#include "task.h"
#include "uart_manager.h"
#include "uwb_port.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_EXPECTED_DEV_ID (0xDECA0302UL)
#define MAX_MESSAGE_LENGTH (MAC_MAX_FRAME_SIZE)
#define DEFAULT_NODE_ADDRESS (0x0001)
#define MAX_PROTOCOL_HANDLERS (8U)
#define UWB_RX_QUEUE_LENGTH (10U)
#define UWB_RX_TASK_PRIORITY (6U)
#define UWB_RX_TASK_STACK_SIZE (768U)
#define UWB_TX_MUTEX_TIMEOUT_MS (2U)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    STATE_OFF,
    STATE_INITIALIZATION,
    STATE_ACTIVE,
    STATE_FAULTED
} uwb_state_E;

typedef enum
{
    FAULT_NONE = 0,
    FAULT_INIT_NULL_DEV,
    FAULT_PROBE_FAILED,
    FAULT_DEVICE_ID_INVALID,
    FAULT_CONFIG_FAILED,
    FAULT_TX_FAILED,
    FAULT_RX_FAILED
} uwb_fault_code_e;

typedef struct
{
    bool fault_present;
    bool init_device_completed;
    bool start_requested;
    bool stop_requested;
} uwb_state_machine_inputs_t;

typedef struct
{
    uint32_t device_id;
    float temperature;
    float voltage;
} uwb_measurements_t;

typedef struct
{
    uint16_t my_pan_id;
    uint16_t my_address;
    uint8_t tx_sequence;
} uwb_addressing_t;

typedef struct
{
    uint8_t protocol_type;
    uwb_protocol_handler_t handler;
} protocol_handler_entry_t;

typedef struct
{
    uint32_t total_received;
    uint32_t unhandled;
    uint32_t invalid;
} protocol_stats_t;

typedef struct
{
    uint8_t data[MAC_MAX_FRAME_SIZE];
    uint16_t length;
    uint64_t rx_timestamp;
} uwb_rx_event_t;

typedef struct
{
    uint32_t tx_mutex_timeouts;
    uint32_t tx_mutex_max_wait_ticks;
    uint32_t rx_queue_overflows;
} uwb_debug_stats_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_device_id(void);
STATIC void uwb_dispatch_protocol_message(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                          uint64_t rx_timestamp);
STATIC void uwb_process_received_message(const uint8_t* data, uint16_t length,
                                         uint64_t rx_timestamp);
STATIC void uwb_rx_task(void* argument);
STATIC void uwb_isr_rx_callback(const uint8_t* data, uint16_t length, uint64_t rx_timestamp);
STATIC void uwb_tx_done_callback(uint64_t tx_timestamp);
STATIC void uwb_state_machine_sample_inputs(void);
STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void uwb_state_initialization_on_entry(uint16_t prevState);
STATIC void uwb_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uwb_init(void);
STATIC void uwb_create_task(void);
STATIC void uwb_process_10Hz(void);

extern const module_S uwb_module;

const module_S uwb_module = {
    .module_name         = "uwb",
    .module_init         = uwb_init,
    .module_create_task  = uwb_create_task,
    .module_process_10Hz = uwb_process_10Hz, // State machine for fault detection
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC protocol_handler_entry_t protocol_handlers[MAX_PROTOCOL_HANDLERS];
STATIC uint8_t protocol_handler_count        = 0;
STATIC protocol_stats_t protocol_stats       = {0};
STATIC uwb_tx_done_handler_t tx_done_handler = NULL;
STATIC uint32_t next_message_id              = 1;
STATIC uint32_t current_tx_message_id        = 0; // ID of message currently being transmitted
STATIC QueueHandle_t uwb_rx_queue            = NULL;
STATIC SemaphoreHandle_t uwb_tx_mutex        = NULL;
STATIC uwb_debug_stats_t debug_stats         = {0};

STATIC const state_s uwb_states[] = {
    [STATE_OFF]            = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_INITIALIZATION] = {.process = NULL,
                              .onEntry = uwb_state_initialization_on_entry,
                              .onExit  = NULL},
    [STATE_ACTIVE]         = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_FAULTED] = {.process = NULL, .onEntry = uwb_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s uwb_state_machine = {.prev_state      = STATE_OFF,
                                            .curr_state      = STATE_OFF,
                                            .next_state      = STATE_OFF,
                                            .timer           = 0,
                                            .transitionLogic = uwb_transition_logic,
                                            .states          = uwb_states};

STATIC uwb_dev_t* uwb_dev                   = NULL;
STATIC uwb_measurements_t measurements      = {0};
STATIC uwb_fault_code_e fault_code          = FAULT_NONE;
STATIC uwb_state_machine_inputs_t sm_inputs = {0};
STATIC uwb_addressing_t addressing          = {
             .my_pan_id   = MAC_DEFAULT_PAN_ID,
             .my_address  = DEFAULT_NODE_ADDRESS,
             .tx_sequence = 0,
};

STATIC uwb_rx_stats_t rx_stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool verify_device_id(void)
{
    if (uwb_dev == NULL)
    {
        return false;
    }

    uwb_port_status_t ret = uwb_port_check_device_id(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        return false;
    }

    measurements.device_id = uwb_port_read_device_id(uwb_dev);

    return (measurements.device_id == UWB_EXPECTED_DEV_ID);
}

STATIC void uwb_init(void)
{
    // Create FreeRTOS resources BEFORE scheduler starts
    uwb_rx_queue = xQueueCreate(UWB_RX_QUEUE_LENGTH, sizeof(uwb_rx_event_t));
    if (uwb_rx_queue == NULL)
    {
        error_handler_fatal("uwb", "Failed to create RX queue");
    }

    // Use binary semaphore (not mutex) since it's given from ISR context
    uwb_tx_mutex = xSemaphoreCreateBinary();
    if (uwb_tx_mutex == NULL)
    {
        error_handler_fatal("uwb", "Failed to create TX semaphore");
    }
    // Initialize as available (mutex starts taken, binary semaphore starts empty)
    xSemaphoreGive(uwb_tx_mutex);
}

STATIC void uwb_create_task(void)
{
    // Create high-priority RX task for deferred interrupt processing
    // Called after scheduler starts
    BaseType_t task_result = xTaskCreate(uwb_rx_task, "uwb_rx", UWB_RX_TASK_STACK_SIZE, NULL,
                                         UWB_RX_TASK_PRIORITY, NULL);
    if (task_result != pdPASS)
    {
        error_handler_fatal("uwb", "Failed to create RX task");
    }

    // Auto-start UWB radio (critical system component)
    uwb_start();
}

STATIC void uwb_state_machine_sample_inputs(void)
{
    sm_inputs.fault_present         = (fault_code != FAULT_NONE);
    sm_inputs.init_device_completed = (uwb_dev != NULL);
}

STATIC void uwb_process_10Hz(void)
{
    uwb_state_machine_sample_inputs();
    state_machine_periodic(&uwb_state_machine);
}

STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    (void)stateTimer; // Unused
    uint16_t nextState = currentState;

    switch (currentState)
    {
        case STATE_OFF:
            if (sm_inputs.start_requested)
            {
                sm_inputs.start_requested = false;
                nextState                 = STATE_INITIALIZATION;
            }
            break;

        case STATE_INITIALIZATION:
            if (sm_inputs.fault_present)
            {
                nextState = STATE_FAULTED;
            }
            else if (sm_inputs.init_device_completed)
            {
                nextState = STATE_ACTIVE;
            }
            break;

        case STATE_ACTIVE:
            if (sm_inputs.fault_present)
            {
                nextState = STATE_FAULTED;
            }
            else if (sm_inputs.stop_requested)
            {
                sm_inputs.stop_requested = false;
                nextState                = STATE_OFF;
            }
            break;

        case STATE_FAULTED:
            if (sm_inputs.stop_requested)
            {
                sm_inputs.stop_requested = false;
                fault_code               = FAULT_NONE;
                nextState                = STATE_OFF;
            }
            break;

        default:
            nextState = STATE_OFF;
            break;
    }

    return nextState;
}

STATIC void uwb_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;

    fault_code = FAULT_NONE;

    uwb_dev = uwb_port_init();
    if (uwb_dev == NULL)
    {
        fault_code = FAULT_INIT_NULL_DEV;
        return;
    }

    uwb_port_status_t ret = uwb_port_probe_and_init(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (!verify_device_id())
    {
        fault_code = FAULT_DEVICE_ID_INVALID;
        return;
    }

    uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
    uwb_port_set_address(uwb_dev, addressing.my_address);

    ret = uwb_port_configure(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        fault_code = FAULT_CONFIG_FAILED;
        return;
    }

    uwb_port_register_isr_callbacks(uwb_dev);
    uwb_port_set_rx_callback(uwb_isr_rx_callback);
    uwb_port_set_tx_done_callback(uwb_tx_done_callback);
    uwb_port_enable_rx_interrupt();

    rx_stats = (uwb_rx_stats_t){0};
}

STATIC void uwb_isr_rx_callback(const uint8_t* data, uint16_t length, uint64_t rx_timestamp)
{
    // Runs at NVIC priority 5 - highest priority that can call FreeRTOS FromISR functions
    // Hardware read and FreeRTOS communication happen atomically in this callback

    if (length > MAC_MAX_FRAME_SIZE)
    {
        return; // Silently drop oversized frames in ISR
    }

    uwb_rx_event_t event;
    event.length       = length;
    event.rx_timestamp = rx_timestamp;
    memcpy(event.data, data, length);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Non-blocking send - drops packet if queue full (prevents ISR blocking)
    if (xQueueSendFromISR(uwb_rx_queue, &event, &xHigherPriorityTaskWoken) != pdPASS)
    {
        // Queue full - packet dropped
        debug_stats.rx_queue_overflows++;
    }

    // Trigger immediate context switch if RX task has higher priority
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

STATIC void uwb_rx_task(void* argument)
{
    (void)argument;

    uwb_rx_event_t event;
    uint32_t loop_count = 0;

    for (;;)
    {
        // Block indefinitely waiting for RX events from ISR
        if (xQueueReceive(uwb_rx_queue, &event, portMAX_DELAY) == pdPASS)
        {
            // Process in task context - safe to call protocol handlers
            uwb_process_received_message(event.data, event.length, event.rx_timestamp);
        }

        // Periodic stack watermark check (every 1000 packets)
        if (++loop_count >= 1000)
        {
            loop_count                  = 0;
            UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
            if (stack_remaining < 128)
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "uwb_rx",
                                  "Low stack: %lu words remaining", (unsigned long)stack_remaining);
            }
        }
    }
}

STATIC void uwb_process_received_message(const uint8_t* data, uint16_t length,
                                         uint64_t rx_timestamp)
{
    // Validate frame has minimum MAC header
    if (length < MAC_FRAME_SHORT_HEADER_SIZE)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Frame too small: %u bytes", length);
        return;
    }

    // Parse MAC frame
    mac_frame_short_t* rx_frame = (mac_frame_short_t*)data;
    uint16_t payload_len        = length - MAC_FRAME_SHORT_HEADER_SIZE;

    if (payload_len > MAC_MAX_PAYLOAD_SIZE)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Payload too large: %u bytes",
                          payload_len);
        return;
    }

    rx_stats.received++;

    // Dispatch to protocol handler with timestamp paired to this message
    uwb_dispatch_protocol_message(rx_frame->payload, payload_len, rx_frame->src_addr, rx_timestamp);
}

STATIC void uwb_tx_done_callback(uint64_t tx_timestamp)
{
    // CRITICAL: This runs in ISR context
    // Release TX mutex now that transmission is complete
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uwb_tx_mutex, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // Forward TX done notification with message ID to registered handler
    if (tx_done_handler != NULL)
    {
        tx_done_handler(current_tx_message_id, tx_timestamp);
    }

    // Clear current message ID
    current_tx_message_id = 0;
}

STATIC void uwb_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    const char* fault_str;
    switch (fault_code)
    {
        case FAULT_INIT_NULL_DEV:
            fault_str = "Device init failed (NULL)";
            break;
        case FAULT_PROBE_FAILED:
            fault_str = "Probe/init failed";
            break;
        case FAULT_DEVICE_ID_INVALID:
            fault_str = "Invalid device ID";
            break;
        case FAULT_CONFIG_FAILED:
            fault_str = "TX/RX config failed";
            break;
        case FAULT_TX_FAILED:
            fault_str = "Transmission failed";
            break;
        case FAULT_RX_FAILED:
            fault_str = "Reception failed";
            break;
        default:
            fault_str = "Unknown fault";
            break;
    }

    error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "%s (code=%u)", fault_str, fault_code);
}

STATIC void uwb_dispatch_protocol_message(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                          uint64_t rx_timestamp)
{
    protocol_stats.total_received++;

    if (data == NULL || length < sizeof(protocol_header_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb",
                          "Invalid protocol message: null data or too short");
        protocol_stats.invalid++;
        return;
    }

    uint8_t protocol_type = data[0];

    if (protocol_type == 0x00 || protocol_type > 0x0F)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Invalid protocol type: 0x%02X",
                          protocol_type);
        protocol_stats.invalid++;
        return;
    }

    bool handled = false;
    for (uint8_t i = 0; i < protocol_handler_count; i++)
    {
        if (protocol_handlers[i].protocol_type == protocol_type)
        {
            protocol_handlers[i].handler(data, length, src_addr, rx_timestamp);
            handled = true;
            break;
        }
    }

    if (!handled)
    {
        protocol_stats.unhandled++;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void uwb_get_status(uwb_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    switch (uwb_state_machine.curr_state)
    {
        case STATE_ACTIVE:
            status->state = UWB_STATE_ACTIVE;
            break;
        case STATE_INITIALIZATION:
            status->state = UWB_STATE_INITIALIZATION;
            break;
        case STATE_OFF:
            status->state = UWB_STATE_OFF;
            break;
        case STATE_FAULTED:
            status->state = UWB_STATE_FAULTED;
            break;
        default:
            status->state = UWB_STATE_OFF;
            break;
    }

    status->device_id   = measurements.device_id;
    status->temperature = measurements.temperature;
    status->voltage     = measurements.voltage;
    status->fault_code  = fault_code;
    status->my_address  = addressing.my_address;
    status->my_pan_id   = addressing.my_pan_id;
}

void uwb_get_rx_stats(uwb_rx_stats_t* stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->received  = rx_stats.received;
    stats->rx_errors = rx_stats.rx_errors;
    stats->filtered  = rx_stats.filtered;
}

void uwb_reset_rx_stats(void)
{
    rx_stats = (uwb_rx_stats_t){0};
}

bool uwb_is_ready(void)
{
    return (uwb_state_machine.curr_state == STATE_ACTIVE);
}

uwb_dev_t* uwb_get_device(void)
{
    return uwb_dev;
}

void uwb_start(void)
{
    sm_inputs.start_requested = true;
}

void uwb_stop(void)
{
    sm_inputs.stop_requested = true;
}

void uwb_set_address(uint16_t address, uint16_t pan_id)
{
    addressing.my_address = address;
    addressing.my_pan_id  = pan_id;

    if (uwb_state_machine.curr_state == STATE_ACTIVE)
    {
        uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
        uwb_port_set_address(uwb_dev, addressing.my_address);
    }
}

uint16_t uwb_get_address(void)
{
    return addressing.my_address;
}

bool uwb_soft_reset(void)
{
    if (uwb_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    uwb_port_status_t ret = uwb_port_soft_reset(uwb_dev);
    return (ret == UWB_PORT_SUCCESS);
}

uwb_send_result_t uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr)
{
    uwb_send_result_t result = {.success = false, .message_id = 0};

    // Validate radio state and parameters
    if (uwb_state_machine.curr_state != STATE_ACTIVE || data == NULL || length == 0 ||
        length > MAC_MAX_PAYLOAD_SIZE)
    {
        return result;
    }

    // Acquire TX mutex to prevent concurrent sends
    TickType_t start_tick = xTaskGetTickCount();
    if (xSemaphoreTake(uwb_tx_mutex, pdMS_TO_TICKS(UWB_TX_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        debug_stats.tx_mutex_timeouts++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "TX mutex timeout");
        return result;
    }
    TickType_t wait_ticks = xTaskGetTickCount() - start_tick;
    if (wait_ticks > debug_stats.tx_mutex_max_wait_ticks)
    {
        debug_stats.tx_mutex_max_wait_ticks = wait_ticks;
    }

    // Generate unique message ID
    uint32_t message_id = next_message_id++;
    if (next_message_id == 0)
        next_message_id = 1; // Avoid ID 0

    uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
    mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;

    frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
    frame->sequence      = addressing.tx_sequence++;
    frame->dest_pan_id   = addressing.my_pan_id;
    frame->dest_addr     = dest_addr;
    frame->src_addr      = addressing.my_address;

    memcpy(frame->payload, data, length);

    uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + length;

    // Store message ID for TX done callback
    current_tx_message_id = message_id;

    uwb_port_status_t port_result = uwb_port_send_message(uwb_dev, tx_buffer, frame_len);

    if (port_result != UWB_PORT_SUCCESS)
    {
        // TX failed - release mutex immediately and clear message ID
        current_tx_message_id = 0;
        xSemaphoreGive(uwb_tx_mutex);
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Send failed: port error %d", port_result);
        return result;
    }

    // Success - mutex held until TX done callback fires
    result.success    = true;
    result.message_id = message_id;
    return result;
}

uwb_send_result_t uwb_send_message_delayed(const uint8_t* data, uint16_t length, uint16_t dest_addr,
                                           uint32_t tx_time_dtuh)
{
    uwb_send_result_t result = {.success = false, .message_id = 0};

    // Validate radio state and parameters
    if (uwb_state_machine.curr_state != STATE_ACTIVE || data == NULL || length == 0 ||
        length > MAC_MAX_PAYLOAD_SIZE)
    {
        return result;
    }

    // Acquire TX mutex to prevent concurrent sends
    TickType_t start_tick = xTaskGetTickCount();
    if (xSemaphoreTake(uwb_tx_mutex, pdMS_TO_TICKS(UWB_TX_MUTEX_TIMEOUT_MS)) != pdTRUE)
    {
        debug_stats.tx_mutex_timeouts++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Delayed TX mutex timeout");
        return result;
    }
    TickType_t wait_ticks = xTaskGetTickCount() - start_tick;
    if (wait_ticks > debug_stats.tx_mutex_max_wait_ticks)
    {
        debug_stats.tx_mutex_max_wait_ticks = wait_ticks;
    }

    // Generate unique message ID
    uint32_t message_id = next_message_id++;
    if (next_message_id == 0)
        next_message_id = 1; // Avoid ID 0

    uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
    mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;

    frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
    frame->sequence      = addressing.tx_sequence++;
    frame->dest_pan_id   = addressing.my_pan_id;
    frame->dest_addr     = dest_addr;
    frame->src_addr      = addressing.my_address;

    memcpy(frame->payload, data, length);

    uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + length;

    // Store message ID for TX done callback
    current_tx_message_id = message_id;

    // Use delayed transmission with the provided absolute TX timestamp
    uwb_port_status_t port_result =
        uwb_port_send_message_delayed(uwb_dev, tx_buffer, frame_len, tx_time_dtuh);

    if (port_result != UWB_PORT_SUCCESS)
    {
        // TX failed - release mutex immediately and clear message ID
        current_tx_message_id = 0;
        xSemaphoreGive(uwb_tx_mutex);
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Delayed send failed: port error %d",
                          port_result);
        return result;
    }

    // Success - mutex held until TX done callback fires
    result.success    = true;
    result.message_id = message_id;
    return result;
}

uint64_t uwb_get_last_tx_timestamp(void)
{
    if (uwb_dev == NULL)
    {
        return 0;
    }

    return uwb_port_get_last_tx_timestamp(uwb_dev);
}

bool uwb_register_protocol_handler(uint8_t protocol_type, uwb_protocol_handler_t handler)
{
    if (handler == NULL)
    {
        return false;
    }

    // Check if protocol already registered - update if found
    for (uint8_t i = 0; i < protocol_handler_count; i++)
    {
        if (protocol_handlers[i].protocol_type == protocol_type)
        {
            protocol_handlers[i].handler = handler;
            return true;
        }
    }

    // Add new handler
    if (protocol_handler_count >= MAX_PROTOCOL_HANDLERS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "Protocol handler table full");
        return false;
    }

    protocol_handlers[protocol_handler_count].protocol_type = protocol_type;
    protocol_handlers[protocol_handler_count].handler       = handler;
    protocol_handler_count++;

    return true;
}

void uwb_unregister_protocol_handler(uint8_t protocol_type)
{
    // Find and remove handler
    for (uint8_t i = 0; i < protocol_handler_count; i++)
    {
        if (protocol_handlers[i].protocol_type == protocol_type)
        {
            // Shift remaining handlers down
            for (uint8_t j = i; j < protocol_handler_count - 1; j++)
            {
                protocol_handlers[j] = protocol_handlers[j + 1];
            }
            protocol_handler_count--;
            memset(&protocol_handlers[protocol_handler_count], 0, sizeof(protocol_handler_entry_t));
            return;
        }
    }
}

void uwb_get_protocol_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid)
{
    if (total_received != NULL)
    {
        *total_received = protocol_stats.total_received;
    }
    if (unhandled != NULL)
    {
        *unhandled = protocol_stats.unhandled;
    }
    if (invalid != NULL)
    {
        *invalid = protocol_stats.invalid;
    }
}

void uwb_reset_protocol_stats(void)
{
    memset(&protocol_stats, 0, sizeof(protocol_stats));
}

void uwb_register_tx_done_handler(uwb_tx_done_handler_t handler)
{
    tx_done_handler = handler;
}
