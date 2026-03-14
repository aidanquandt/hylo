/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb.h"
#include "FreeRTOS.h"
#include "common.h"
#include "mac_802154.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "protocol.pb.h"
#include "task_config.h"
#include "gpio_driver.h"
#include "timer_driver.h"
#include "semphr.h"
#include "state_machine.h"
#include "task.h"
#include "uwb_driver.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_EXPECTED_DEV_ID (0xDECA0302UL)
#define MAX_MESSAGE_LENGTH (MAC_MAX_FRAME_SIZE)
#define DEFAULT_NODE_ADDRESS (0x0001U) // Changed from 0x0000 (reserved/coordinator address)
#define DEFAULT_PAN_ID MAC_DEFAULT_PAN_ID
#define MAX_PROTOCOL_HANDLERS (8U)
#define UWB_TX_QUEUE_DEPTH_WARNING (8U)
#define UWB_FAULT_RECOVERY_MS (100U) // Hardware recovery is instant if it will work
#define UWB_MAX_RETRY_ATTEMPTS (3U)

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
    uwb_tx_complete_handler_t complete_handler;
} tx_context_t;

typedef struct
{
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
STATIC void uwb_tx_done_from_driver(uint32_t message_id, uint64_t tx_timestamp);
STATIC void uwb_state_machine_sample_inputs(void);
STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void uwb_state_off_on_entry(uint16_t prevState);
STATIC void uwb_state_initialization_on_entry(uint16_t prevState);
STATIC void uwb_state_retry_on_entry(uint16_t prevState);
STATIC void uwb_state_faulted_on_entry(uint16_t prevState);
STATIC bool uwb_is_transient_fault(uwb_fault_code_e fault);

/*---------------------------------------------------------------------------
 * Private Function Prototypes (module-internal)
 *---------------------------------------------------------------------------*/
STATIC void uwb_process_100Hz(void);
STATIC void uwb_100Hz_task(void* pvParameters);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC SemaphoreHandle_t uwb_hw_mutex = NULL;
STATIC protocol_context_t protocol = {0};
STATIC tx_context_t tx             = {0};
STATIC rx_context_t rx             = {0};
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

    uwb_driver_status_t ret = uwb_driver_check_device_id(uwb_dev);
    if (ret != UWB_DRIVER_SUCCESS)
    {
        return false;
    }

    device_id = uwb_driver_read_device_id(uwb_dev);

    return (device_id == UWB_EXPECTED_DEV_ID);
}

STATIC void uwb_100Hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        uwb_process_100Hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

void uwb_init(void)
{
    uwb_hw_mutex = xSemaphoreCreateBinary();
    if (uwb_hw_mutex == NULL)
    {
        system_halt("uwb", "Failed to create UWB hardware mutex");
    }
    xSemaphoreGive(uwb_hw_mutex);

    uwb_start();

    BaseType_t result = xTaskCreate(uwb_100Hz_task, "uwb_100hz", TASK_STACK_2KB,
                                    NULL, TASK_PRIORITY_UWB_STATE_MACHINE, NULL);
    if (result != pdPASS)
    {
        system_halt("uwb", "Failed to create 100Hz task");
    }
}

STATIC void uwb_state_machine_sample_inputs(void)
{
    state.inputs.fault_present           = (state.fault_code != FAULT_NONE);
    state.inputs.transient_fault_present = uwb_is_transient_fault(state.fault_code);
    state.inputs.init_device_completed   = (uwb_dev != NULL);
}

STATIC void uwb_process_100Hz(void)
{
    uwb_state_machine_sample_inputs();
    state_machine_periodic(&uwb_state_machine);
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

    uwb_dev = uwb_driver_init();
    if (uwb_dev == NULL)
    {
        state.fault_code = FAULT_INIT_NULL_DEV;
        return;
    }

    uwb_driver_status_t ret = uwb_driver_probe_and_init(uwb_dev);
    if (ret != UWB_DRIVER_SUCCESS)
    {
        state.fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (!verify_device_id())
    {
        state.fault_code = FAULT_DEVICE_ID_INVALID;
        return;
    }

    uwb_driver_set_pan_id(uwb_dev, addressing.my_pan_id);
    uwb_driver_set_address(uwb_dev, addressing.my_address);

    ret = uwb_driver_configure(uwb_dev);
    if (ret != UWB_DRIVER_SUCCESS)
    {
        state.fault_code = FAULT_CONFIG_FAILED;
        return;
    }

    uwb_driver_register_isr_callbacks(uwb_dev);
    uwb_driver_set_rx_callback(uwb_process_received_message);
    uwb_driver_set_tx_done_callback(uwb_tx_done_from_driver);
    uwb_driver_enable_rx_interrupt();

    rx.stats = (uwb_rx_stats_t){0};
}

STATIC void uwb_tx_done_from_driver(uint32_t message_id, uint64_t tx_timestamp)
{
    if (tx.complete_handler != NULL)
    {
        tx.complete_handler(message_id, tx_timestamp);
    }
}

STATIC void uwb_process_received_message(const uint8_t* data, uint16_t length,
                                         uint64_t rx_timestamp)
{
    // Validate frame has minimum MAC header
    if (length < MAC_FRAME_SHORT_HEADER_SIZE)
    {
        UwbFrameTooSmallEvent ev = UwbFrameTooSmallEvent_init_zero;
        ev.length = length;
        protocol_tx_UwbFrameTooSmallEvent(&ev);
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
        UwbPayloadTooLargeEvent ev = UwbPayloadTooLargeEvent_init_zero;
        ev.payload_len = payload_len;
        protocol_tx_UwbPayloadTooLargeEvent(&ev);
        return;
    }

    rx.stats.received++;

    // Dispatch to protocol handler with timestamp paired to this message
    uwb_dispatch_protocol_message(rx_frame->payload, payload_len, rx_frame->src_addr, rx_timestamp);
}

STATIC void uwb_state_retry_on_entry(uint16_t prevState)
{
    (void)prevState;
    UwbRetryAttemptEvent ev = UwbRetryAttemptEvent_init_zero;
    ev.current = state.init_retry_count;
    ev.max    = UWB_MAX_RETRY_ATTEMPTS;
    protocol_tx_UwbRetryAttemptEvent(&ev);

    uwb_dev = NULL;
    uwb_driver_reset_tx_queue();
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

    UwbFaultManualStopRequiredEvent ev = UwbFaultManualStopRequiredEvent_init_zero;
    strncpy(ev.fault_str, fault_str, sizeof(ev.fault_str) - 1);
    ev.fault_str[sizeof(ev.fault_str) - 1] = '\0';
    ev.fault_code = state.fault_code;
    protocol_tx_UwbFaultManualStopRequiredEvent(&ev);

    uwb_dev = NULL;
    uwb_driver_reset_tx_queue();
}

STATIC void uwb_dispatch_protocol_message(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                          uint64_t rx_timestamp)
{
    protocol.stats.total_received++;

    if (data == NULL || length < sizeof(protocol_header_t))
    {
        UwbInvalidProtocolMessageEvent ev = UwbInvalidProtocolMessageEvent_init_zero;
        protocol_tx_UwbInvalidProtocolMessageEvent(&ev);
        protocol.stats.invalid++;
        return;
    }

    uint8_t protocol_type = data[0];

    if (protocol_type == 0x00 || protocol_type > 0x0F)
    {
        UwbInvalidProtocolTypeEvent ev = UwbInvalidProtocolTypeEvent_init_zero;
        ev.protocol_type = protocol_type;
        protocol_tx_UwbInvalidProtocolTypeEvent(&ev);
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
    return uwb_driver_get_tx_queue_overflows();
}

uint32_t uwb_get_rx_queue_overflows(void)
{
    return uwb_driver_get_rx_queue_overflows();
}

uint32_t uwb_get_tx_timeouts(void)
{
    return debug_stats.tx_timeouts;
}

uint32_t uwb_get_tx_queue_depth(void)
{
    return uwb_driver_get_tx_queue_depth();
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
        uwb_driver_set_pan_id(uwb_dev, addressing.my_pan_id);
        uwb_driver_set_address(uwb_dev, addressing.my_address);
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
    uwb_driver_status_t ret = uwb_driver_soft_reset(uwb_dev);
    xSemaphoreGive(uwb_hw_mutex);

    return (ret == UWB_DRIVER_SUCCESS);
}

uwb_send_result_t uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr)
{
    uwb_send_result_t result = {.success = false, .message_id = 0};

    if (uwb_state_machine.curr_state != UWB_STATE_ACTIVE || data == NULL || length == 0 ||
        length > MAC_MAX_PAYLOAD_SIZE)
    {
        return result;
    }

    uint32_t message_id = uwb_driver_send_async(data, length, dest_addr);
    if (message_id == 0)
    {
        UwbTxQueueFullEvent ev = UwbTxQueueFullEvent_init_zero;
        protocol_tx_UwbTxQueueFullEvent(&ev);
        return result;
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
        UwbInvalidProtocolTypeEvent ev = UwbInvalidProtocolTypeEvent_init_zero;
        ev.protocol_type = protocol_type;
        protocol_tx_UwbInvalidProtocolTypeEvent(&ev);
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
        UwbProtocolHandlerTableFullEvent ev = UwbProtocolHandlerTableFullEvent_init_zero;
        protocol_tx_UwbProtocolHandlerTableFullEvent(&ev);
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

void uwb_get_driver_statistics(uwb_driver_stats_t* out)
{
    if (out == NULL)
    {
        return;
    }
    uwb_driver_statistics_t drv = uwb_driver_get_statistics();
    out->rx_ok_count       = drv.rx_ok_count;
    out->rx_timeout_count  = drv.rx_timeout_count;
    out->tx_done_count     = drv.tx_done_count;
    out->send_message_count = drv.send_message_count;
    out->PHE               = drv.PHE;
    out->RSL               = drv.RSL;
    out->CRCG              = drv.CRCG;
    out->CRCB              = drv.CRCB;
    out->ARFE              = drv.ARFE;
    out->OVER              = drv.OVER;
    out->SFDTO             = drv.SFDTO;
    out->PTO               = drv.PTO;
    out->RTO               = drv.RTO;
    out->TXF               = drv.TXF;
    out->irq_status        = drv.irq_status;
    out->status_lo         = drv.status_lo;
    out->status_hi         = drv.status_hi;
}

void uwb_register_tx_complete_handler(uwb_tx_complete_handler_t handler)
{
    tx.complete_handler = handler;
}
