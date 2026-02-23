/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "mac_802154.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_timer.h"
#include "queue.h"
#include "semphr.h"
#include "state_machine.h"
#include "task.h"
#include "task_config.h"
#include "uart_manager.h"
#include "uwb_port.h"
#include "uwb_protocol_messages.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_EXPECTED_DEV_ID (0xDECA0302UL)
#define MAX_MESSAGE_LENGTH (MAC_MAX_FRAME_SIZE)
#define DEFAULT_NODE_ADDRESS (0x0001U) // Changed from 0x0000 (reserved/coordinator address)
#define DEFAULT_PAN_ID MAC_DEFAULT_PAN_ID
#define MAX_PROTOCOL_HANDLERS (8U)
#define UWB_RX_QUEUE_LENGTH (20U) // Increased for burst tolerance during TWR exchanges
#define UWB_RX_TASK_STACK_SIZE (768U)
#define UWB_TX_QUEUE_LENGTH (16U)
#define UWB_TX_TASK_STACK_SIZE (512U)
#define UWB_SM_TASK_STACK_SIZE (256U) // State machine task
#define UWB_TX_TIMEOUT_MS (3U)
#define UWB_TX_TIMEOUT_THRESHOLD (2U)
#define UWB_TX_QUEUE_DEPTH_WARNING (8U)
#define UWB_FAULT_RECOVERY_MS (100U) // Hardware recovery is instant if it will work
#define UWB_MAX_RETRY_ATTEMPTS (3U)
#define UWB_RX_TASK_STACK_CHECK_INTERVAL (1000U)
#define UWB_RX_TASK_STACK_MIN_WORDS (128U)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
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
    bool transient_fault_present;
    bool init_device_completed;
    bool start_requested;
    bool stop_requested;
} uwb_state_machine_inputs_t;

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
    uint8_t payload[MAC_MAX_PAYLOAD_SIZE];
    uint16_t length;
    uint16_t dest_addr;
    uint32_t message_id;
} uwb_tx_queue_item_t;

typedef struct
{
    uint32_t tx_queue_overflows;
    uint32_t rx_queue_overflows;
    uint32_t tx_timeouts;
} uwb_debug_stats_t;

typedef struct
{
    protocol_handler_entry_t handlers[MAX_PROTOCOL_HANDLERS];
    uint8_t handler_count;
    protocol_stats_t stats;
} protocol_context_t;

typedef struct
{
    QueueHandle_t queue;
    TaskHandle_t task_handle; // For ISR → Task notification
    uwb_tx_complete_handler_t complete_handler;
    uint32_t next_message_id;
    volatile uint32_t current_message_id;   // Written by task, read by ISR callback validation
    volatile uint64_t current_tx_timestamp; // Written by ISR, read by task
    uint8_t consecutive_timeouts;
} tx_context_t;

typedef struct
{
    QueueHandle_t queue;
    uwb_rx_stats_t stats;
} rx_context_t;

typedef struct
{
    uwb_fault_code_e fault_code;
    uint8_t init_retry_count;
    uwb_state_machine_inputs_t inputs;
} state_context_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_device_id(void);
STATIC void uwb_dispatch_protocol_message(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                          uint64_t rx_timestamp);
STATIC void uwb_process_received_message(const uint8_t* data, uint16_t length,
                                         uint64_t rx_timestamp);
STATIC void uwb_rx_task(void* argument);
STATIC void uwb_tx_task(void* argument);
STATIC void uwb_isr_rx_callback(const uint8_t* data, uint16_t length, uint64_t rx_timestamp);
STATIC void uwb_tx_complete_callback(uint64_t tx_timestamp);
STATIC void uwb_state_machine_sample_inputs(void);
STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void uwb_state_off_on_entry(uint16_t prevState);
STATIC void uwb_state_initialization_on_entry(uint16_t prevState);
STATIC void uwb_state_retry_on_entry(uint16_t prevState);
STATIC void uwb_state_faulted_on_entry(uint16_t prevState);
STATIC bool uwb_is_transient_fault(uwb_fault_code_e fault);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uwb_init(void);
STATIC void uwb_create_tasks(void);
STATIC void uwb_state_machine_task(void* argument);

extern const module_S uwb_module;

const module_S uwb_module = {
    .module_name         = "uwb",
    .module_init         = uwb_init,
    .module_create_tasks = uwb_create_tasks,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC SemaphoreHandle_t uwb_hw_mutex = NULL;
STATIC protocol_context_t protocol    = {0};
STATIC tx_context_t tx                = {
                   .next_message_id = 1, // Start at 1, avoid 0
};
STATIC rx_context_t rx       = {0};
STATIC state_context_t state = {
    .fault_code = FAULT_NONE,
};

STATIC const state_s uwb_states[] = {
    [UWB_STATE_OFF]            = {.onEntry = uwb_state_off_on_entry},
    [UWB_STATE_INITIALIZATION] = {.onEntry = uwb_state_initialization_on_entry},
    [UWB_STATE_ACTIVE]         = {0},
    [UWB_STATE_RETRY]          = {.onEntry = uwb_state_retry_on_entry},
    [UWB_STATE_FAULTED]        = {.onEntry = uwb_state_faulted_on_entry},
};

STATIC state_machine_s uwb_state_machine = {
    .prev_state      = UWB_STATE_OFF,
    .curr_state      = UWB_STATE_OFF,
    .next_state      = UWB_STATE_OFF,
    .timer           = 0,
    .transitionLogic = uwb_transition_logic,
    .states          = uwb_states,
};

STATIC uwb_dev_t* uwb_dev          = NULL;
STATIC uint32_t device_id          = 0;
STATIC uwb_addressing_t addressing = {
    .my_pan_id   = DEFAULT_PAN_ID,
    .my_address  = DEFAULT_NODE_ADDRESS,
    .tx_sequence = 0,
};

STATIC uwb_debug_stats_t debug_stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool uwb_is_transient_fault(uwb_fault_code_e fault)
{
    return (fault == FAULT_PROBE_FAILED || fault == FAULT_TX_FAILED || fault == FAULT_RX_FAILED);
}

STATIC void uwb_state_off_on_entry(uint16_t prevState)
{
    (void)prevState;

    uwb_dev                = NULL;
    state.fault_code       = FAULT_NONE;
    state.init_retry_count = 0;
}

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

    device_id = uwb_port_read_device_id(uwb_dev);

    return (device_id == UWB_EXPECTED_DEV_ID);
}

STATIC void uwb_init(void)
{
    // Create RX queue
    rx.queue = xQueueCreate(UWB_RX_QUEUE_LENGTH, sizeof(uwb_rx_event_t));
    if (rx.queue == NULL)
    {
        error_handler_fatal("uwb", "Failed to create RX queue");
    }

    // Create TX queue
    tx.queue = xQueueCreate(UWB_TX_QUEUE_LENGTH, sizeof(uwb_tx_queue_item_t));
    if (tx.queue == NULL)
    {
        error_handler_fatal("uwb", "Failed to create TX queue");
    }

    // Hardware mutex for serializing all hardware access and protecting addressing state
    uwb_hw_mutex = xSemaphoreCreateBinary();
    if (uwb_hw_mutex == NULL)
    {
        error_handler_fatal("uwb", "Failed to create UWB hardware mutex");
    }

    // Initialize to available state (count = 1)
    xSemaphoreGive(uwb_hw_mutex);
}

STATIC void uwb_create_tasks(void)
{
    // Create high-priority RX task for deferred interrupt processing
    BaseType_t task_result = xTaskCreate(uwb_rx_task, "uwb_rx", UWB_RX_TASK_STACK_SIZE, NULL,
                                         TASK_PRIORITY_UWB_RX, NULL);
    if (task_result != pdPASS)
    {
        error_handler_fatal("uwb", "Failed to create RX task");
    }

    // Create TX task for serialized transmission
    task_result = xTaskCreate(uwb_tx_task, "uwb_tx", UWB_TX_TASK_STACK_SIZE, NULL,
                              TASK_PRIORITY_UWB_TX, NULL);
    if (task_result != pdPASS)
    {
        error_handler_fatal("uwb", "Failed to create TX task");
    }

    // Create state machine task for fault detection
    task_result = xTaskCreate(uwb_state_machine_task, "uwb_sm", UWB_SM_TASK_STACK_SIZE, NULL,
                              TASK_PRIORITY_UWB_TX - 1, NULL); // Just below TX priority
    if (task_result != pdPASS)
    {
        error_handler_fatal("uwb", "Failed to create state machine task");
    }

    // Auto-start UWB radio (critical system component)
    uwb_start();
}

STATIC void uwb_state_machine_sample_inputs(void)
{
    state.inputs.fault_present           = (state.fault_code != FAULT_NONE);
    state.inputs.transient_fault_present = uwb_is_transient_fault(state.fault_code);
    state.inputs.init_device_completed   = (uwb_dev != NULL);
}

/**
 * @brief UWB state machine task - monitors and manages radio state
 *
 * Runs at 100Hz to detect faults and manage radio lifecycle.
 */
STATIC void uwb_state_machine_task(void* argument)
{
    (void)argument;

    TickType_t lastWake     = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100Hz = 10ms

    watchdog_register_task(50); // Expect heartbeat every 50ms (5x period)

    for (;;)
    {
        uwb_state_machine_sample_inputs();
        state_machine_periodic(&uwb_state_machine);

        watchdog_heartbeat();
        vTaskDelayUntil(&lastWake, period);
    }
}

STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;

    switch (currentState)
    {
        case UWB_STATE_OFF:
            if (state.inputs.start_requested)
            {
                state.inputs.start_requested = false;
                nextState                    = UWB_STATE_INITIALIZATION;
            }
            break;

        case UWB_STATE_INITIALIZATION:
            if (state.inputs.fault_present)
            {
                // Transient faults (TX/RX) get retries, others go straight to FAULTED
                if (state.init_retry_count < UWB_MAX_RETRY_ATTEMPTS &&
                    state.inputs.transient_fault_present)
                {
                    state.init_retry_count++;
                    nextState = UWB_STATE_RETRY;
                }
                else
                {
                    nextState = UWB_STATE_FAULTED;
                }
            }
            else if (state.inputs.init_device_completed)
            {
                state.init_retry_count = 0;
                nextState              = UWB_STATE_ACTIVE;
            }
            break;

        case UWB_STATE_ACTIVE:
            if (state.inputs.fault_present)
            {
                // Transient faults (TX/RX) get retries, others go straight to FAULTED
                if (state.init_retry_count < UWB_MAX_RETRY_ATTEMPTS &&
                    state.inputs.transient_fault_present)
                {
                    state.init_retry_count++;
                    nextState = UWB_STATE_RETRY;
                }
                else
                {
                    nextState = UWB_STATE_FAULTED;
                }
            }
            else if (state.inputs.stop_requested)
            {
                state.inputs.stop_requested = false;
                nextState                   = UWB_STATE_OFF;
            }
            break;

        case UWB_STATE_RETRY:
            if (state.inputs.stop_requested)
            {
                state.inputs.stop_requested = false;
                nextState                   = UWB_STATE_OFF;
            }
            else if (stateTimer > MS_TO_100HZ_TICKS(UWB_FAULT_RECOVERY_MS))
            {
                nextState = UWB_STATE_INITIALIZATION;
            }
            break;

        case UWB_STATE_FAULTED:
            if (state.inputs.stop_requested)
            {
                state.inputs.stop_requested = false;
                nextState                   = UWB_STATE_OFF;
            }
            break;

        default:
            nextState = UWB_STATE_OFF;
            break;
    }

    return nextState;
}

STATIC void uwb_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;

    state.fault_code = FAULT_NONE;

    uwb_dev = uwb_port_init();
    if (uwb_dev == NULL)
    {
        state.fault_code = FAULT_INIT_NULL_DEV;
        return;
    }

    uwb_port_status_t ret = uwb_port_probe_and_init(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        state.fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (!verify_device_id())
    {
        state.fault_code = FAULT_DEVICE_ID_INVALID;
        return;
    }

    uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
    uwb_port_set_address(uwb_dev, addressing.my_address);

    ret = uwb_port_configure(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        state.fault_code = FAULT_CONFIG_FAILED;
        return;
    }

    uwb_port_register_isr_callbacks(uwb_dev);
    uwb_port_set_rx_callback(uwb_isr_rx_callback);
    uwb_port_set_tx_done_callback(uwb_tx_complete_callback);
    uwb_port_enable_rx_interrupt();

    rx.stats = (uwb_rx_stats_t){0};
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

    if (xQueueSendFromISR(rx.queue, &event, &xHigherPriorityTaskWoken) != pdPASS)
    {
        debug_stats.rx_queue_overflows++;

        // Log dropped protocol for diagnostics
        if (length >= sizeof(protocol_header_t))
        {
            static const char* protocol_names[] = {
                [PROTOCOL_TYPE_TWR]        = "TWR",
                [PROTOCOL_TYPE_DATA]       = "DATA",
                [PROTOCOL_TYPE_OTA_CONFIG] = "OTA_CONFIG",
            };

            uint8_t protocol_type = data[0];
            const char* name =
                (protocol_type < sizeof(protocol_names) / sizeof(protocol_names[0]) &&
                 protocol_names[protocol_type] != NULL)
                    ? protocol_names[protocol_type]
                    : "UNKNOWN";

            uart_manager_print("[ISR_DROP] RX Queue full: Protocol=%s (0x%02X) - Total=%lu\r\n",
                               name, protocol_type, (unsigned long)debug_stats.rx_queue_overflows);
        }
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
        if (xQueueReceive(rx.queue, &event, portMAX_DELAY) == pdPASS)
        {
            // Process in task context - safe to call protocol handlers
            uwb_process_received_message(event.data, event.length, event.rx_timestamp);
        }

        // Periodic stack watermark check
        if (++loop_count >= UWB_RX_TASK_STACK_CHECK_INTERVAL)
        {
            loop_count                  = 0;
            UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
            if (stack_remaining < UWB_RX_TASK_STACK_MIN_WORDS)
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
    const mac_frame_short_t* rx_frame = (const mac_frame_short_t*)data;

    // Filter non-data frames (beacons, ACKs, commands)
    uint16_t frame_type = rx_frame->frame_control & MAC_FC_TYPE_MASK;
    if (frame_type != MAC_FC_TYPE_DATA)
    {
        rx.stats.filtered++;
        return;
    }

    uint16_t payload_len = length - MAC_FRAME_SHORT_HEADER_SIZE;

    if (payload_len > MAC_MAX_PAYLOAD_SIZE)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Payload too large: %u bytes",
                          payload_len);
        return;
    }

    rx.stats.received++;

    // Dispatch to protocol handler with timestamp paired to this message
    uwb_dispatch_protocol_message(rx_frame->payload, payload_len, rx_frame->src_addr, rx_timestamp);
}

STATIC void uwb_tx_complete_callback(uint64_t tx_timestamp)
{
    // CRITICAL: This runs in ISR context - only store timestamp and notify task
    // Handler invocation happens in TX task context to avoid blocking operations in ISR
    tx.current_tx_timestamp = tx_timestamp;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(tx.task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

STATIC void uwb_tx_task(void* argument)
{
    (void)argument;

    uwb_tx_queue_item_t tx_item;

    for (;;)
    {
        // Producer-Consumer Pattern: Wait for work, THEN acquire resource
        // Do NOT take mutex before queue receive - would deadlock waiting for message
        // while holding mutex, blocking all hardware access (set_address, soft_reset, etc)
        if (xQueueReceive(tx.queue, &tx_item, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        // Drop messages if radio not ready
        if (uwb_state_machine.curr_state != UWB_STATE_ACTIVE)
        {
            if (tx.complete_handler != NULL)
            {
                tx.complete_handler(tx_item.message_id, 0);
            }
            continue;
        }

        // Acquire hardware NOW (only when we have work and radio is ready)
        // Mutex held through entire TX sequence: build → transmit → complete
        // This ensures atomicity: addressing state used to build frame remains valid
        // until HW completes transmission and we correlate the timestamp
        xSemaphoreTake(uwb_hw_mutex, portMAX_DELAY);

        // Clear stale notifications from late ISR
        ulTaskNotifyTake(pdTRUE, 0);

        // Build MAC frame (reads addressing.*, must be inside mutex)
        uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
        mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;

        frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
        frame->sequence      = addressing.tx_sequence++;
        frame->dest_pan_id   = addressing.my_pan_id;
        frame->dest_addr     = tx_item.dest_addr;
        frame->src_addr      = addressing.my_address;

        memcpy(frame->payload, tx_item.payload, tx_item.length);
        uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + tx_item.length;

        // Store message ID for validation
        tx.current_message_id = tx_item.message_id;

        // Transmit
        uwb_port_status_t result = uwb_port_send_message(uwb_dev, tx_buffer, frame_len);
        if (result != UWB_PORT_SUCCESS)
        {
            tx.current_message_id = 0;
            xSemaphoreGive(uwb_hw_mutex);

            error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "TX failed: port error %d", result);
            if (tx.complete_handler != NULL)
            {
                tx.complete_handler(tx_item.message_id, 0);
            }
            continue;
        }

        // Wait for ISR completion signal (~300µs typical, 3ms timeout = 10x margin)
        // NOTE: Mutex held during wait - this is CORRECT because:
        //   1. Hardware is physically busy transmitting (can't be interrupted)
        //   2. Prevents address changes mid-TX (frame built with current address)
        //   3. ISR just signals, doesn't manipulate mutex (ISRs shouldn't block)
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UWB_TX_TIMEOUT_MS)) == 0)
        {
            // Timeout - hardware may be stuck
            debug_stats.tx_timeouts++;
            tx.consecutive_timeouts++;
            tx.current_message_id = 0;
            xSemaphoreGive(uwb_hw_mutex);

            error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "TX timeout msg %lu",
                              (unsigned long)tx_item.message_id);
            if (tx.complete_handler != NULL)
            {
                tx.complete_handler(tx_item.message_id, 0);
            }

            // Fault detection: consecutive timeouts indicate hardware lockup
            if (tx.consecutive_timeouts >= UWB_TX_TIMEOUT_THRESHOLD)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "Hardware stuck after %u timeouts",
                                  tx.consecutive_timeouts);
                tx.consecutive_timeouts = 0;
                xQueueReset(tx.queue);
                state.fault_code = FAULT_TX_FAILED;
            }
            continue;
        }

        // TX completed - validate message ID and capture timestamp
        tx.consecutive_timeouts       = 0;
        uint32_t completed_message_id = tx.current_message_id;
        uint64_t timestamp            = tx.current_tx_timestamp;
        tx.current_message_id         = 0;

        // Release mutex AFTER reading timestamp - entire TX sequence now complete
        xSemaphoreGive(uwb_hw_mutex);

        // Validate ID (detect late ISR from timed-out TX)
        if (completed_message_id != tx_item.message_id)
        {
            error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "ID mismatch: expected %lu, got %lu",
                              (unsigned long)tx_item.message_id,
                              (unsigned long)completed_message_id);
            timestamp = 0; // Mark as invalid
        }

        // Notify application (outside mutex - user code may block)
        if (tx.complete_handler != NULL)
        {
            tx.complete_handler(tx_item.message_id, timestamp);
        }
    }
}

STATIC void uwb_state_retry_on_entry(uint16_t prevState)
{
    (void)prevState;
    error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Retry attempt %u/%u", state.init_retry_count,
                      UWB_MAX_RETRY_ATTEMPTS);

    // Clean up from failed initialization
    uwb_dev = NULL;
    xQueueReset(tx.queue);
}

STATIC void uwb_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    static const char* fault_strings[] = {
        [FAULT_NONE]              = "None",
        [FAULT_INIT_NULL_DEV]     = "Device init failed (NULL)",
        [FAULT_PROBE_FAILED]      = "Probe/init failed",
        [FAULT_DEVICE_ID_INVALID] = "Invalid device ID",
        [FAULT_CONFIG_FAILED]     = "TX/RX config failed",
        [FAULT_TX_FAILED]         = "Transmission hardware fault",
        [FAULT_RX_FAILED]         = "Reception failed",
    };

    const char* fault_str = (state.fault_code < sizeof(fault_strings) / sizeof(fault_strings[0]))
                                ? fault_strings[state.fault_code]
                                : "Unknown fault";

    error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "%s (code=%u) - manual stop required", fault_str,
                      state.fault_code);

    // Clean up device state
    uwb_dev = NULL;
    xQueueReset(tx.queue);
}

STATIC void uwb_dispatch_protocol_message(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                          uint64_t rx_timestamp)
{
    protocol.stats.total_received++;

    if (data == NULL || length < sizeof(protocol_header_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb",
                          "Invalid protocol message: null data or too short");
        protocol.stats.invalid++;
        return;
    }

    uint8_t protocol_type = data[0];

    if (protocol_type == 0x00 || protocol_type > 0x0F)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "Invalid protocol type: 0x%02X",
                          protocol_type);
        protocol.stats.invalid++;
        return;
    }

    bool handled = false;
    for (uint8_t i = 0; i < protocol.handler_count; i++)
    {
        if (protocol.handlers[i].protocol_type == protocol_type)
        {
            protocol.handlers[i].handler(data, length, src_addr, rx_timestamp);
            handled = true;
            break;
        }
    }

    if (!handled)
    {
        protocol.stats.unhandled++;
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

    status->state      = (uwb_state_e)uwb_state_machine.curr_state;
    status->device_id  = device_id;
    status->fault_code = state.fault_code;

    // Atomic read of addressing state (protected by mutex)
    xSemaphoreTake(uwb_hw_mutex, portMAX_DELAY);
    status->my_address = addressing.my_address;
    status->my_pan_id  = addressing.my_pan_id;
    xSemaphoreGive(uwb_hw_mutex);
}

void uwb_get_rx_stats(uwb_rx_stats_t* stats)
{
    if (stats == NULL)
    {
        return;
    }

    *stats = rx.stats;
}

void uwb_reset_rx_stats(void)
{
    rx.stats = (uwb_rx_stats_t){0};
}

uint32_t uwb_get_tx_queue_overflows(void)
{
    return debug_stats.tx_queue_overflows;
}

uint32_t uwb_get_rx_queue_overflows(void)
{
    return debug_stats.rx_queue_overflows;
}

uint32_t uwb_get_tx_timeouts(void)
{
    return debug_stats.tx_timeouts;
}

uint32_t uwb_get_tx_queue_depth(void)
{
    if (tx.queue == NULL)
    {
        return 0;
    }
    return (uint32_t)uxQueueMessagesWaiting(tx.queue);
}

bool uwb_is_ready(void)
{
    return (uwb_state_machine.curr_state == UWB_STATE_ACTIVE);
}

void uwb_start(void)
{
    state.inputs.start_requested = true;
}

void uwb_stop(void)
{
    state.inputs.stop_requested = true;
}

void uwb_set_address(uint16_t address, uint16_t pan_id)
{
    xSemaphoreTake(uwb_hw_mutex, portMAX_DELAY);

    addressing.my_address = address;
    addressing.my_pan_id  = pan_id;

    if (uwb_state_machine.curr_state == UWB_STATE_ACTIVE)
    {
        uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
        uwb_port_set_address(uwb_dev, addressing.my_address);
    }

    xSemaphoreGive(uwb_hw_mutex);
}

uint16_t uwb_get_address(void)
{
    // Atomic read on ARM Cortex-M7 (16-bit load is single instruction)
    return addressing.my_address;
}

uint16_t uwb_get_pan_id(void)
{
    // Atomic read on ARM Cortex-M7 (16-bit load is single instruction)
    return addressing.my_pan_id;
}

bool uwb_soft_reset(void)
{
    if (uwb_state_machine.curr_state != UWB_STATE_ACTIVE)
    {
        return false;
    }

    xSemaphoreTake(uwb_hw_mutex, portMAX_DELAY);
    uwb_port_status_t ret = uwb_port_soft_reset(uwb_dev);
    xSemaphoreGive(uwb_hw_mutex);

    return (ret == UWB_PORT_SUCCESS);
}

uwb_send_result_t uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr)
{
    uwb_send_result_t result = {.success = false, .message_id = 0};

    if (uwb_state_machine.curr_state != UWB_STATE_ACTIVE || data == NULL || length == 0 ||
        length > MAC_MAX_PAYLOAD_SIZE)
    {
        return result;
    }

    // Generate unique message ID (atomic increment, avoid 0)
    taskENTER_CRITICAL();
    uint32_t message_id = tx.next_message_id++;
    if (tx.next_message_id == 0)
    {
        tx.next_message_id = 1;
    }
    taskEXIT_CRITICAL();

    uwb_tx_queue_item_t tx_item = {
        .length     = length,
        .dest_addr  = dest_addr,
        .message_id = message_id,
    };
    memcpy(tx_item.payload, data, length);

    if (xQueueSend(tx.queue, &tx_item, 0) != pdPASS)
    {
        debug_stats.tx_queue_overflows++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "TX queue full");
        return result;
    }

    // Warn if queue depth is high
    UBaseType_t queue_waiting = uxQueueMessagesWaiting(tx.queue);
    if (queue_waiting > UWB_TX_QUEUE_DEPTH_WARNING)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb", "TX queue depth: %lu items",
                          (unsigned long)queue_waiting);
    }

    result.success    = true;
    result.message_id = message_id;
    return result;
}

bool uwb_register_protocol_handler(uint8_t protocol_type, uwb_protocol_handler_t handler)
{
    if (handler == NULL)
    {
        return false;
    }

    // Validate protocol type range (per uwb_protocol_messages.h)
    if (protocol_type == 0 || protocol_type > 0x0F)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "Invalid protocol type: 0x%02X",
                          protocol_type);
        return false;
    }

    // Update existing handler if already registered
    for (uint8_t i = 0; i < protocol.handler_count; i++)
    {
        if (protocol.handlers[i].protocol_type == protocol_type)
        {
            protocol.handlers[i].handler = handler;
            return true;
        }
    }

    // Add new handler
    if (protocol.handler_count >= MAX_PROTOCOL_HANDLERS)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "uwb", "Protocol handler table full");
        return false;
    }

    protocol.handlers[protocol.handler_count].protocol_type = protocol_type;
    protocol.handlers[protocol.handler_count].handler       = handler;
    protocol.handler_count++;

    return true;
}

void uwb_unregister_protocol_handler(uint8_t protocol_type)
{
    for (uint8_t i = 0; i < protocol.handler_count; i++)
    {
        if (protocol.handlers[i].protocol_type == protocol_type)
        {
            // Shift remaining handlers down
            for (uint8_t j = i; j < protocol.handler_count - 1; j++)
            {
                protocol.handlers[j] = protocol.handlers[j + 1];
            }
            protocol.handler_count--;
            memset(&protocol.handlers[protocol.handler_count], 0, sizeof(protocol_handler_entry_t));
            return;
        }
    }
}

void uwb_get_protocol_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid)
{
    if (total_received != NULL)
    {
        *total_received = protocol.stats.total_received;
    }
    if (unhandled != NULL)
    {
        *unhandled = protocol.stats.unhandled;
    }
    if (invalid != NULL)
    {
        *invalid = protocol.stats.invalid;
    }
}

void uwb_reset_protocol_stats(void)
{
    protocol.stats = (protocol_stats_t){0};
}

void uwb_register_tx_complete_handler(uwb_tx_complete_handler_t handler)
{
    tx.complete_handler = handler;
}
