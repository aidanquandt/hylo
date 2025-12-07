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
#include "uart_cmd_router.h"
#include "uart_manager.h"
#include "uwb_port.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_EXPECTED_DEV_ID (0xDECA0302UL)      // DW3000 Device ID
#define UWB_DEFAULT_CHANNEL UWB_CHANNEL_5       // Default UWB channel
#define MAX_MESSAGE_LENGTH (MAC_MAX_FRAME_SIZE) // Max UWB frame size

// Default address (can be changed at runtime with uwb_set_address)
#define DEFAULT_NODE_ADDRESS (0x0001)

// State machine states
typedef enum
{
    STATE_OFF,            // Radio off, waiting for start command (default)
    STATE_INITIALIZATION, // Initializing hardware
    STATE_ACTIVE,         // Normal operation (TX/RX)
    STATE_FAULTED         // Error state - initialization or communication failed
} uwb_state_E;

// Fault codes
typedef enum
{
    FAULT_NONE = 0,
    FAULT_INIT_NULL_DEV,     // Device init returned NULL
    FAULT_PROBE_FAILED,      // Probe and init failed
    FAULT_DEVICE_ID_INVALID, // Device ID verification failed
    FAULT_CONFIG_FAILED,     // TX/RX configuration failed
    FAULT_TX_FAILED,         // Transmission failed
    FAULT_RX_FAILED          // Reception failed
} uwb_fault_code_e;

typedef struct
{
    bool fault_present;
    bool init_device_completed;
    bool start_requested; // User commanded start via UART
    bool stop_requested;  // User commanded stop via UART
} uwb_state_machine_inputs_t;

// UWB measurement data
typedef struct
{
    uint32_t device_id;
    float temperature;
    float voltage;
} uwb_measurements_t;

// 802.15.4 addressing configuration
typedef struct
{
    uint16_t my_pan_id;
    uint16_t my_address;
    uint8_t tx_sequence;
} uwb_addressing_t;

// RX callback function type
typedef void (*uwb_rx_callback_t)(const uint8_t* data, uint16_t length, uint16_t src_addr);

// RX statistics
typedef struct
{
    uint32_t received;  // Valid frames received and delivered to callback
    uint32_t rx_errors; // Hardware RX errors
    uint32_t filtered;  // Frames filtered (wrong address/PAN)
} uwb_rx_stats_t;

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
STATIC bool uwb_cmd_handler(const cmd_parsed_t* parsed);

const module_S uwb_module = {
    .module_name = "uwb",
    .module_init = uwb_init,
    .module_process_1Hz = uwb_process_1Hz,
    .module_process_100Hz = uwb_process_100Hz,
    .module_cmd_handler = uwb_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
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

// Hardware state
STATIC uwb_dev_t* uwb_dev = NULL;
STATIC uwb_measurements_t measurements = {0};
STATIC uwb_fault_code_e fault_code = FAULT_NONE;
STATIC uwb_state_machine_inputs_t sm_inputs = {0};
STATIC uwb_addressing_t addressing = {
    .my_pan_id = MAC_DEFAULT_PAN_ID,
    .my_address = DEFAULT_NODE_ADDRESS,
    .tx_sequence = 0,
};

// RX statistics and callback
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

    // Check the device ID through port layer
    uwb_port_status_t ret = uwb_port_check_device_id(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        return false;
    }

    // Read and store device ID
    measurements.device_id = uwb_port_read_device_id(uwb_dev);

    // Verify it matches expected value (DW3000)
    return (measurements.device_id == UWB_EXPECTED_DEV_ID);
}

STATIC void read_measurements(void)
{
    if (uwb_dev == NULL)
    {
        return;
    }

    // Read and store IC temperature and voltage (single optimized call)
    uwb_port_read_temp_and_voltage(uwb_dev, &measurements.temperature, &measurements.voltage);
}

STATIC void uwb_state_machine_sample_inputs(void)
{
    sm_inputs.fault_present = (fault_code != FAULT_NONE);
    sm_inputs.init_device_completed = (uwb_dev != NULL);
    // start_requested and stop_requested are set by UART commands
}

STATIC void uwb_init(void)
{
    // State machine is already initialized with static values
}

STATIC void uwb_process_1Hz(void)
{
    // Toggle LED to show we're running
    platform_gpio_toggle_led_green();

    // Sample state machine inputs
    uwb_state_machine_sample_inputs();

    // Run state machine at 1Hz
    state_machine_periodic(&uwb_state_machine);

    // Read measurements at 1Hz (temperature, voltage)
    if (uwb_state_machine.curr_state == STATE_ACTIVE)
    {
        read_measurements();
    }
}

STATIC void uwb_process_100Hz(void)
{
    // Run active process at 100Hz for responsive RX polling
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
            // Wait for start command from user
            if (sm_inputs.start_requested)
            {
                sm_inputs.start_requested = false; // Clear flag
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
            else
            {
                // stay in INITIALIZATION state
            }
            break;

        case STATE_ACTIVE:
            if (sm_inputs.fault_present)
            {
                nextState = STATE_FAULTED;
            }
            else if (sm_inputs.stop_requested)
            {
                sm_inputs.stop_requested = false; // Clear flag
                nextState = STATE_OFF;
            }
            else
            {
                // stay in ACTIVE state
            }
            break;

        case STATE_FAULTED:
            // Can recover to OFF state with stop command
            if (sm_inputs.stop_requested)
            {
                sm_inputs.stop_requested = false; // Clear flag
                fault_code = FAULT_NONE;          // Clear fault
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
    (void)prevState; // Unused

    // Clear any previous fault
    fault_code = FAULT_NONE;

    // Get the port device structure
    uwb_dev = uwb_port_init();
    if (uwb_dev == NULL)
    {
        fault_code = FAULT_INIT_NULL_DEV;
        return;
    }

    // Probe and initialize the device
    uwb_port_status_t ret = uwb_port_probe_and_init(uwb_dev);
    if (ret != UWB_PORT_SUCCESS)
    {
        fault_code = FAULT_PROBE_FAILED;
        return;
    }

    // Verify device ID
    if (!verify_device_id())
    {
        fault_code = FAULT_DEVICE_ID_INVALID;
        return;
    }

    // Set 802.15.4 addressing
    uwb_port_set_pan_id(uwb_dev, addressing.my_pan_id);
    uwb_port_set_address(uwb_dev, addressing.my_address);

    // Configure radio for bidirectional communication (all nodes configured the same)
    ret = uwb_port_configure(uwb_dev, UWB_DEFAULT_CHANNEL);
    if (ret != UWB_PORT_SUCCESS)
    {
        fault_code = FAULT_CONFIG_FAILED;
        return;
    }

    // Initialize RX statistics
    rx_stats = (uwb_rx_stats_t){0};

    // Read initial measurements
    read_measurements();
}

STATIC void uwb_state_active_process(void)
{
    // All nodes continuously listen for incoming messages (polled at 100Hz)
    uint8_t rx_buffer[MAX_MESSAGE_LENGTH];
    uint16_t received = 0;
    uwb_port_status_t rx_result =
        uwb_port_receive_message(uwb_dev, rx_buffer, MAX_MESSAGE_LENGTH, &received);

    if (rx_result == UWB_PORT_SUCCESS)
    {
        // Check minimum frame size (12 bytes header minimum)
        if (received >= MAC_FRAME_SHORT_HEADER_SIZE)
        {
            mac_frame_short_t* rx_frame = (mac_frame_short_t*)rx_buffer;

            // Verify this frame is for us (check dest address and PAN ID)
            if ((rx_frame->dest_addr == addressing.my_address ||
                 rx_frame->dest_addr == MAC_BROADCAST_ADDR) &&
                rx_frame->dest_pan_id == addressing.my_pan_id)
            {
                rx_stats.received++;

                // Invoke RX callback if registered
                if (rx_callback != NULL)
                {
                    // Extract payload (skip MAC header)
                    uint16_t payload_len = received - MAC_FRAME_SHORT_HEADER_SIZE;

                    // Safety: Validate payload size before invoking callback
                    if (payload_len <= MAC_MAX_PAYLOAD_SIZE)
                    {
                        rx_callback(rx_frame->payload, payload_len, rx_frame->src_addr);
                    }
                }
            }
            else
            {
                // Frame not for us (filtered by address/PAN)
                rx_stats.filtered++;
            }
        }
    }
    else if (rx_result != UWB_PORT_ERROR_NO_DATA)
    {
        // Real RX error (not just "no data available")
        rx_stats.rx_errors++;
        fault_code = FAULT_RX_FAILED;
    }

    // Note: Application modules call uwb_send_message() to transmit
}

STATIC void uwb_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState; // Unused

    // Log fault information
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

bool uwb_device_id(void)
{
    // For backward compatibility - verify device ID is correct
    return (measurements.device_id == UWB_EXPECTED_DEV_ID);
}

uint32_t uwb_get_device_id(void)
{
    return measurements.device_id;
}

float uwb_get_temperature(void)
{
    return measurements.temperature;
}

float uwb_get_voltage(void)
{
    return measurements.voltage;
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
    // Check if device is initialized and ready
    if (uwb_dev == NULL || !uwb_is_ready())
    {
        return false;
    }

    // Perform soft reset through port layer
    uwb_port_status_t ret = uwb_port_soft_reset(uwb_dev);
    return (ret == UWB_PORT_SUCCESS);
}

bool uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr)
{
    // Check if device is ready
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

    // Validate parameters
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

    // Build 802.15.4 MAC frame
    uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
    mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;

    frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
    frame->sequence = addressing.tx_sequence++;
    frame->dest_pan_id = addressing.my_pan_id;
    frame->dest_addr = dest_addr;
    frame->src_addr = addressing.my_address;

    // Copy payload
    memcpy(frame->payload, data, length);

    // Send frame (header + payload)
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

/*---------------------------------------------------------------------------
 * Command Handler
 *---------------------------------------------------------------------------*/
STATIC bool uwb_cmd_handler(const cmd_parsed_t* parsed)
{
    switch (parsed->action)
    {
        case CMD_ACTION_GET:
            if (strcmp(parsed->target, "status") == 0)
            {
                const char* state_str;
                switch (uwb_state_machine.curr_state)
                {
                    case STATE_ACTIVE:
                        state_str = "active";
                        break;
                    case STATE_INITIALIZATION:
                        state_str = "init";
                        break;
                    case STATE_OFF:
                        state_str = "off";
                        break;
                    case STATE_FAULTED:
                        state_str = "FAULTED";
                        break;
                    default:
                        state_str = "unknown";
                        break;
                }
                uart_manager_print("UWB status: %s, dev_id=0x%08X", state_str,
                                   (unsigned int)measurements.device_id);
                if (fault_code != FAULT_NONE)
                {
                    uart_manager_print(", fault_code=%u", fault_code);
                }
                uart_manager_print("\r\n");
                return true;
            }
            else if (strcmp(parsed->target, "addr") == 0)
            {
                uart_manager_print("UWB addr: PAN=0x%04X, ADDR=0x%04X\r\n", addressing.my_pan_id,
                                   addressing.my_address);
                return true;
            }
            else if (strcmp(parsed->target, "temp") == 0)
            {
                if (uwb_state_machine.curr_state != STATE_ACTIVE)
                {
                    uart_manager_print("UWB not active yet\r\n");
                    return true;
                }
                uart_manager_print("UWB temp: %.2f C\r\n", measurements.temperature);
                return true;
            }
            else if (strcmp(parsed->target, "voltage") == 0)
            {
                if (uwb_state_machine.curr_state != STATE_ACTIVE)
                {
                    uart_manager_print("UWB not active yet\r\n");
                    return true;
                }
                uart_manager_print("UWB voltage: %.2f V\r\n", measurements.voltage);
                return true;
            }
            else if (strcmp(parsed->target, "stats") == 0)
            {
                uart_manager_print("UWB RX stats:\r\n");
                uart_manager_print("  Received:  %u frames\r\n", (unsigned int)rx_stats.received);
                uart_manager_print("  Errors:    %u\r\n", (unsigned int)rx_stats.rx_errors);
                uart_manager_print("  Filtered:  %u (wrong addr/PAN)\r\n",
                                   (unsigned int)rx_stats.filtered);

                // Calculate and display success rate
                uint32_t total_attempts = rx_stats.received + rx_stats.rx_errors;
                if (total_attempts > 0)
                {
                    float success_rate = (100.0f * rx_stats.received) / total_attempts;
                    uart_manager_print("  Success:   %.1f%%\r\n", success_rate);
                }

                uart_manager_print("  Callback:  %s\r\n", rx_callback ? "registered" : "none");
                uart_manager_print("\r\nNote: Use 'beacon.get stats' for TX statistics\r\n");
                return true;
            }
            break;

        case CMD_ACTION_SET:
            // No SET commands currently supported
            break;

        case CMD_ACTION_REQ:
            if (strcmp(parsed->target, "start") == 0)
            {
                // Start UWB radio: "uwb.req start"
                if (uwb_state_machine.curr_state == STATE_OFF)
                {
                    sm_inputs.start_requested = true;
                    uart_manager_print("UWB start requested\r\n");
                    return true;
                }
                else if (uwb_state_machine.curr_state == STATE_ACTIVE)
                {
                    uart_manager_print("UWB already active\r\n");
                    return true;
                }
                else
                {
                    uart_manager_print("UWB not in OFF state (current: %d)\r\n",
                                       uwb_state_machine.curr_state);
                    return true;
                }
            }
            else if (strcmp(parsed->target, "stop") == 0)
            {
                // Stop UWB radio: "uwb.req stop"
                sm_inputs.stop_requested = true;
                uart_manager_print("UWB stop requested\r\n");
                return true;
            }
            else if (strcmp(parsed->target, "resetstats") == 0)
            {
                // Reset RX statistics: "uwb.req resetstats"
                rx_stats = (uwb_rx_stats_t){0};
                uart_manager_print("RX statistics reset\r\n");
                return true;
            }
            break;

        case CMD_ACTION_UNKNOWN:
        default:
            break;
    }

    return false;
}
