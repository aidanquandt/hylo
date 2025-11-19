/*---------------------------------------------------------------------------
 * @file    dw3000_test.c
 * @brief   DW3000 hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "dw3000_test.h"
#include "module.h"
#include "platform_gpio.h"
#include "dw3000_port.h"
#include "state_machine.h"
#include "mac_802154.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
// Configuration: Define either DW3000_TX_MODE or DW3000_RX_MODE (not both)
// #define DW3000_TX_MODE                          // This device is transmitter
#define DW3000_RX_MODE                       // This device is receiver

#ifdef DW3000_TX_MODE
    #define MY_ADDRESS          (0x0001)        // TX device address
    #define DEST_ADDRESS        (0x0002)        // Send to RX device
#else
    #define MY_ADDRESS          (0x0002)        // RX device address
    #define DEST_ADDRESS        (0x0001)        // Not used in RX mode
#endif

#define DW3000_EXPECTED_DEV_ID  (0xDECA0302UL)  // DW3000 Device ID
#define STARTUP_DELAY_MS        (2000U)            // Wait 2 seconds before probing
#define UWB_DEFAULT_CHANNEL     (5U)            // Default UWB channel
#define MAX_MESSAGE_LENGTH      (MAC_MAX_FRAME_SIZE)  // Max UWB frame size

// State machine states
typedef enum {
    STATE_STARTUP,              // Waiting for startup delay
    STATE_INITIALIZATION,       // Initializing hardware
    STATE_ACTIVE                // Normal operation (TX/RX)
} dw3000_state_E;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_device_id(void);
STATIC void read_measurements(void);

// State machine transition logic
STATIC uint16_t dw3000_test_transition_logic(uint16_t currentState, uint32_t stateTimer);

// State handlers
STATIC void dw3000_test_state_startup_process(void);
STATIC void dw3000_test_state_initialization_on_entry(uint16_t prevState);
STATIC void dw3000_test_state_active_process(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void dw3000_test_init(void);
STATIC void dw3000_test_process_1Hz(void);

extern const module_S dw3000_test_module;
const module_S dw3000_test_module = {
    // .module_init = dw3000_test_init,
    // .module_process_1Hz = dw3000_test_process_1Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s dw3000_states[] = {
    [STATE_STARTUP] = {
        .process = dw3000_test_state_startup_process,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_INITIALIZATION] = {
        .process = NULL,
        .onEntry = dw3000_test_state_initialization_on_entry,
        .onExit = NULL
    },
    [STATE_ACTIVE] = {
        .process = dw3000_test_state_active_process,
        .onEntry = NULL,
        .onExit = NULL
    }
};

STATIC state_machine_s dw3000_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = dw3000_test_transition_logic,
    .states = dw3000_states
};

// Hardware state
STATIC uint32_t device_id = 0;
STATIC float temperature = 0.0f;
STATIC float voltage = 0.0f;
STATIC bool hardware_ready = false;

// 802.15.4 addressing
STATIC uint16_t my_pan_id = MAC_DEFAULT_PAN_ID;     // This device's PAN ID
STATIC uint16_t my_address = MY_ADDRESS;        // This device's short address (from define)
STATIC uint16_t tx_dest_addr = DEST_ADDRESS;    // Destination address (from define)
#ifdef DW3000_TX_MODE
STATIC uint8_t tx_sequence = 0;                 // Frame sequence number
#endif

// RX state
STATIC uint8_t rx_buffer[MAX_MESSAGE_LENGTH];
STATIC uint16_t rx_length = 0;
STATIC uint32_t rx_parsed_value = 0;   // Parsed number from received message (watch in debugger)
STATIC uint32_t rx_count = 0;          // Number of messages received
STATIC uint32_t rx_checks = 0;         // Number of RX checks (debug)

// TX state
STATIC uint32_t tx_attempts = 0;       // Number of TX attempts (debug)
#ifdef DW3000_TX_MODE
STATIC uint16_t tx_counter = 0;        // Incrementing counter 0-6699
#endif

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool verify_device_id(void)
{
    // Check the device ID through port layer
    int ret = dw3000_port_check_device_id();
    if (ret != DW3000_SUCCESS) {
        return false;
    }
    
    // Read and store device ID
    device_id = dw3000_port_read_device_id();
    
    // Verify it matches expected value
    return (device_id == DW3000_EXPECTED_DEV_ID);
}

STATIC void read_measurements(void)
{
    // Read and store IC temperature and voltage (single optimized call)
    dw3000_port_read_temp_and_voltage(&temperature, &voltage);
}

STATIC void dw3000_test_init(void)
{
    // State machine is already initialized with static values
}

STATIC void dw3000_test_process_1Hz(void)
{
    // Toggle LED to show we're running
    platform_gpio_toggle_led_green();
    
    // Read measurements if hardware is ready
    if (hardware_ready) {
        read_measurements();
    }
    
    // Run state machine at 1Hz
    state_machine_periodic(&dw3000_state_machine);
}

STATIC uint16_t dw3000_test_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition to INITIALIZATION after startup delay
            if (stateTimer >= MS_TO_S(STARTUP_DELAY_MS)) {
                nextState = STATE_INITIALIZATION;
            }
            break;
            
        case STATE_INITIALIZATION:
            // Transition to ACTIVE if hardware is ready
            if (hardware_ready) {
                nextState = STATE_ACTIVE;
            }
            break;
            
        case STATE_ACTIVE:
            // Stay in ACTIVE state
            nextState = STATE_ACTIVE;
            break;
            
        default:
            nextState = STATE_STARTUP;
            break;
    }
    
    return nextState;
}

STATIC void dw3000_test_state_startup_process(void)
{
    // Nothing to do - timer automatically increments
}

STATIC void dw3000_test_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;  // Unused
    
    // Get the port chip structure
    dwchip_t* dw_chip = dw3000_port_init();
    if (dw_chip == NULL) {
        return;
    }
    
    // Probe and initialize the device
    int ret = dw3000_port_probe_and_init(dw_chip);
    if (ret != DW3000_SUCCESS) {
        return;
    }
    
    // Verify device ID
    if (!verify_device_id()) {
        return;
    }
    
    hardware_ready = true;
    
    // Set 802.15.4 addressing
    dw3000_port_set_pan_id(my_pan_id);
    dw3000_port_set_address(my_address);
    
    // Configure TX or RX mode based on compile-time define
#ifdef DW3000_TX_MODE
    dw3000_port_configure_tx(UWB_DEFAULT_CHANNEL);
    tx_attempts = 0;
#else
    dw3000_port_configure_rx(UWB_DEFAULT_CHANNEL);
    rx_length = 0;
    rx_parsed_value = 0;
    rx_count = 0;
    rx_checks = 0;
#endif
    
    // Read initial measurements
    read_measurements();
}

STATIC void dw3000_test_state_active_process(void)
{
#ifdef DW3000_TX_MODE
    // TX mode: send 802.15.4 frame with number 68
    tx_attempts++;
    
    // Build 802.15.4 MAC frame
    uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
    mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;
    
    frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;;
    frame->sequence = tx_sequence++;
    frame->dest_pan_id = my_pan_id;
    frame->dest_addr = tx_dest_addr;
    frame->src_addr = my_address;
    
    // Add payload - incrementing counter 0-6699
    char msg[5];
    snprintf(msg, sizeof(msg), "%u", tx_counter);
    uint16_t payload_len = strlen(msg);
    memcpy(frame->payload, msg, payload_len);
    
    // Increment counter (0-6699 wrap around)
    tx_counter++;
    if (tx_counter > 6699) {
        tx_counter = 0;
    }
    
    // Send frame (header + payload)
    uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + payload_len;
    dw3000_port_send_message(tx_buffer, frame_len);
#else
    // RX mode: check for received 802.15.4 frames and parse
    {
        rx_checks++;
        uint16_t received = 0;
        if (dw3000_port_receive_message(rx_buffer, MAX_MESSAGE_LENGTH, &received) == DW3000_SUCCESS) {
            // Check minimum frame size (12 bytes header minimum)
            if (received >= MAC_FRAME_SHORT_HEADER_SIZE) {
                mac_frame_short_t *rx_frame = (mac_frame_short_t*)rx_buffer;
                
                // Verify this frame is for us (check dest address and PAN ID)
                if ((rx_frame->dest_addr == my_address || rx_frame->dest_addr == MAC_BROADCAST_ADDR) &&
                    rx_frame->dest_pan_id == my_pan_id) {
                    
                    rx_length = received;
                    rx_count++;
                    
                    // Extract and parse payload
                    uint16_t payload_len = received - MAC_FRAME_SHORT_HEADER_SIZE;
                    
                    if (payload_len > 0 && payload_len < MAC_MAX_PAYLOAD_SIZE) {
                        rx_frame->payload[payload_len] = '\0';
                        rx_parsed_value = (uint32_t)atoi((char*)rx_frame->payload);
                    }
                }
            }
        }
    }
#endif
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool dw3000_test_device_id(void)
{
    // For backward compatibility - verify device ID is correct
    return (device_id == DW3000_EXPECTED_DEV_ID);
}

uint32_t dw3000_test_get_device_id(void)
{
    return device_id;
}

float dw3000_test_get_temperature(void)
{
    return temperature;
}

float dw3000_test_get_voltage(void)
{
    return voltage;
}

bool dw3000_test_is_ready(void)
{
    return hardware_ready;
}

uint16_t dw3000_test_get_received_message(char *buffer, uint16_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return 0;
    }
    
    uint16_t copy_len = (rx_length < buffer_size - 1) ? rx_length : (buffer_size - 1);
    if (copy_len > 0) {
        memcpy(buffer, rx_buffer, copy_len);
        buffer[copy_len] = '\0';
    }
    
    return rx_length;
}

uint32_t dw3000_test_get_rx_parsed_value(void)
{
    return rx_parsed_value;
}

uint32_t dw3000_test_get_rx_count(void)
{
    return rx_count;
}

uint32_t dw3000_test_get_tx_attempts(void)
{
    return tx_attempts;
}

uint32_t dw3000_test_get_rx_checks(void)
{
    return rx_checks;
}

bool dw3000_test_is_tx_mode(void)
{
#ifdef DW3000_TX_MODE
    return true;
#else
    return false;
#endif
}

bool dw3000_test_is_rx_mode(void)
{
#ifdef DW3000_TX_MODE
    return false;
#else
    return true;
#endif
}

void dw3000_test_set_address(uint16_t address, uint16_t pan_id)
{
    my_address = address;
    my_pan_id = pan_id;
    
    // Update hardware registers if already initialized
    if (hardware_ready) {
        dw3000_port_set_pan_id(my_pan_id);
        dw3000_port_set_address(my_address);
    }
}

void dw3000_test_set_dest_address(uint16_t dest_addr)
{
    tx_dest_addr = dest_addr;
}