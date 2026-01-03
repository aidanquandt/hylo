/*---------------------------------------------------------------------------
 * @file    uwb.c
 * @brief   UWB hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb.h"
#include "error_handler.h"
#include "mac_802154.h"
#include "module.h"
#include "platform_gpio.h"
#include "state_machine.h"
#include "uart_manager.h"
#include "uwb_port.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_EXPECTED_DEV_ID (0xDECA0302UL)
#define UWB_DEFAULT_CHANNEL UWB_CHANNEL_5
#define MAX_MESSAGE_LENGTH (MAC_MAX_FRAME_SIZE)
#define DEFAULT_NODE_ADDRESS (0x0001)

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

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_device_id(void);
STATIC void read_measurements(void);
STATIC void uwb_state_machine_sample_inputs(void);
STATIC uint16_t uwb_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void uwb_state_initialization_on_entry(uint16_t prevState);
STATIC void uwb_state_active_process(void);
STATIC void uwb_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uwb_init(void);
STATIC void uwb_process_1Hz(void);
STATIC void uwb_process_100Hz(void);

extern const module_S uwb_module;

const module_S uwb_module = {
    .module_name = "uwb",
    .module_init = uwb_init,
    .module_process_1Hz = uwb_process_1Hz,
    .module_process_100Hz = uwb_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC const state_s uwb_states[] = {
    [STATE_OFF] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_INITIALIZATION] = {.process = NULL,
                              .onEntry = uwb_state_initialization_on_entry,
                              .onExit = NULL},
    [STATE_ACTIVE] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_FAULTED] = {.process = NULL, .onEntry = uwb_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s uwb_state_machine = {.prev_state = STATE_OFF,
                                            .curr_state = STATE_OFF,
                                            .next_state = STATE_OFF,
                                            .timer = 0,
                                            .transitionLogic = uwb_transition_logic,
                                            .states = uwb_states};

STATIC uwb_dev_t* uwb_dev = NULL;
STATIC uwb_measurements_t measurements = {0};
STATIC uwb_fault_code_e fault_code = FAULT_NONE;
STATIC uwb_state_machine_inputs_t sm_inputs = {0};
STATIC uwb_addressing_t addressing = {
    .my_pan_id = MAC_DEFAULT_PAN_ID,
    .my_address = DEFAULT_NODE_ADDRESS,
    .tx_sequence = 0,
};

STATIC uwb_rx_stats_t rx_stats = {0};
STATIC uwb_rx_callback_t rx_callback = NULL;

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

STATIC void read_measurements(void)
{
    if (uwb_dev == NULL)
    {
        return;
    }

    uwb_port_read_temp_and_voltage(uwb_dev, &measurements.temperature, &measurements.voltage);
}

STATIC void uwb_state_machine_sample_inputs(void)
{
    sm_inputs.fault_present = (fault_code != FAULT_NONE);
    sm_inputs.init_device_completed = (uwb_dev != NULL);
}

STATIC void uwb_init(void)
{
}

STATIC void uwb_process_1Hz(void)
{
    platform_gpio_toggle_led_green();

    uwb_state_machine_sample_inputs();
    state_machine_periodic(&uwb_state_machine);

    if (uwb_state_machine.curr_state == STATE_ACTIVE)
    {
        read_measurements();
    }
}

STATIC void uwb_process_100Hz(void)
{
    if (uwb_state_machine.curr_state == STATE_ACTIVE)
    {
        uwb_state_active_process();
    }
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
                nextState = STATE_INITIALIZATION;
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
                nextState = STATE_OFF;
            }
            break;

        case STATE_FAULTED:
            if (sm_inputs.stop_requested)
            {
                sm_inputs.stop_requested = false;
                fault_code = FAULT_NONE;
                nextState = STATE_OFF;
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

    ret = uwb_port_configure(uwb_dev, UWB_DEFAULT_CHANNEL);
    if (ret != UWB_PORT_SUCCESS)
    {
        fault_code = FAULT_CONFIG_FAILED;
        return;
    }

    rx_stats = (uwb_rx_stats_t){0};
    read_measurements();
}

STATIC void uwb_state_active_process(void)
{
    uint8_t rx_buffer[MAX_MESSAGE_LENGTH];
    uint16_t received = 0;
    uwb_port_status_t rx_result =
        uwb_port_receive_message(uwb_dev, rx_buffer, MAX_MESSAGE_LENGTH, &received);

    if (rx_result == UWB_PORT_SUCCESS)
    {
        if (received >= MAC_FRAME_SHORT_HEADER_SIZE)
        {
            mac_frame_short_t* rx_frame = (mac_frame_short_t*)rx_buffer;

            if ((rx_frame->dest_addr == addressing.my_address ||
                 rx_frame->dest_addr == MAC_BROADCAST_ADDR) &&
                rx_frame->dest_pan_id == addressing.my_pan_id)
            {
                rx_stats.received++;

                if (rx_callback != NULL)
                {
                    uint16_t payload_len = received - MAC_FRAME_SHORT_HEADER_SIZE;

                    if (payload_len <= MAC_MAX_PAYLOAD_SIZE)
                    {
                        rx_callback(rx_frame->payload, payload_len, rx_frame->src_addr);
                    }
                }
            }
            else
            {
                rx_stats.filtered++;
            }
        }
    }
    else if (rx_result != UWB_PORT_ERROR_NO_DATA)
    {
        rx_stats.rx_errors++;
        fault_code = FAULT_RX_FAILED;
    }
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

    status->device_id = measurements.device_id;
    status->temperature = measurements.temperature;
    status->voltage = measurements.voltage;
    status->fault_code = fault_code;
    status->my_address = addressing.my_address;
    status->my_pan_id = addressing.my_pan_id;
}

void uwb_get_rx_stats(uwb_rx_stats_t* stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->received = rx_stats.received;
    stats->rx_errors = rx_stats.rx_errors;
    stats->filtered = rx_stats.filtered;
}

void uwb_reset_rx_stats(void)
{
    rx_stats = (uwb_rx_stats_t){0};
}

bool uwb_is_ready(void)
{
    return (uwb_state_machine.curr_state == STATE_ACTIVE);
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
    addressing.my_pan_id = pan_id;

    // Update hardware registers if in active state
    if (uwb_state_machine.curr_state == STATE_ACTIVE && uwb_dev != NULL)
    {
        uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
        uwb_port_set_address(uwb_dev, addressing.my_address);
    }
}

bool uwb_soft_reset(void)
{
    if (uwb_dev == NULL || !uwb_is_ready())
    {
        return false;
    }

    uwb_port_status_t ret = uwb_port_soft_reset(uwb_dev);
    return (ret == UWB_PORT_SUCCESS);
}

bool uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr)
{
    if (!uwb_is_ready())
    {
        uart_manager_print("[UWB] Send failed: not ready\r\n");
        return false;
    }

    if (uwb_dev == NULL)
    {
        uart_manager_print("[UWB] Send failed: dev is NULL\r\n");
        return false;
    }

    if (data == NULL)
    {
        uart_manager_print("[UWB] Send failed: data is NULL\r\n");
        return false;
    }

    if (length == 0)
    {
        uart_manager_print("[UWB] Send failed: length is 0\r\n");
        return false;
    }

    if (length > MAC_MAX_PAYLOAD_SIZE)
    {
        uart_manager_print("[UWB] Send failed: length too large (%u > %u)\r\n", length,
                           MAC_MAX_PAYLOAD_SIZE);
        return false;
    }

    uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
    mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;

    frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
    frame->sequence = addressing.tx_sequence++;
    frame->dest_pan_id = addressing.my_pan_id;
    frame->dest_addr = dest_addr;
    frame->src_addr = addressing.my_address;

    memcpy(frame->payload, data, length);

    uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + length;
    uart_manager_print("[UWB] Sending: src=0x%04X dst=0x%04X len=%u\r\n", addressing.my_address,
                       dest_addr, frame_len);
    uwb_port_status_t result = uwb_port_send_message(uwb_dev, tx_buffer, frame_len);

    if (result != UWB_PORT_SUCCESS)
    {
        uart_manager_print("[UWB] Port send failed: %d\r\n", result);
        return false;
    }

    return true;
}

void uwb_register_rx_callback(uwb_rx_callback_t callback)
{
    rx_callback = callback;
}
