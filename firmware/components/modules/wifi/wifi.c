/*---------------------------------------------------------------------------
 * @file    wifi.c
 * @brief   WiFi telemetry module for IMU data transmission via ESP8266
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "wifi.h"
#include "module.h"
#include "platform_uart.h"
#include "state_machine.h"
#include "uart_manager.h"
#include "uart_cmd_router.h"
#include <string.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ESP8266_BAUD_RATE           (115200U)
#define STARTUP_DELAY_MS            (3000U)  // ESP8266 boot time
#define AT_COMMAND_TIMEOUT_MS       (2000U)  // AT command response timeout

// State machine states
typedef enum {
    STATE_STARTUP,              // Waiting for ESP8266 boot
    STATE_INITIALIZATION,       // Sending AT commands to configure
    STATE_CONNECTING,           // Connecting to WiFi network
    STATE_ACTIVE,               // Normal operation - sending telemetry
    STATE_FAULTED               // Error state - connection lost or init failed
} wifi_state_E;

// Fault codes
typedef enum {
    FAULT_NONE = 0,
    FAULT_ESP_NO_RESPONSE,      // ESP8266 not responding to AT commands
    FAULT_WIFI_CONNECT_FAILED,  // Failed to connect to WiFi network
    FAULT_TCP_CONNECT_FAILED,   // Failed to establish TCP connection
    FAULT_SEND_FAILED           // Data transmission failed
} wifi_fault_code_e;

typedef struct {
    bool fault_present;
    bool esp_ready;
    bool wifi_connected;
    bool tcp_connected;
} wifi_state_machine_inputs_t;

// WiFi configuration (TODO: Move to config file or commands)
typedef struct {
    char ssid[32];
    char password[64];
    char server_ip[16];
    uint16_t server_port;
} wifi_config_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool send_at_command(const char *cmd, const char *expected_response, uint32_t timeout_ms);
STATIC bool wifi_init_esp8266(void);
STATIC bool wifi_connect_to_network(void);
STATIC bool wifi_establish_tcp_connection(void);
STATIC void wifi_send_telemetry_packet(void);
STATIC void wifi_state_machine_sample_inputs(void);
STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void wifi_state_initialization_on_entry(uint16_t prevState);
STATIC void wifi_state_connecting_on_entry(uint16_t prevState);
STATIC void wifi_state_active_process(void);
STATIC void wifi_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void wifi_init(void);
STATIC void wifi_process_10Hz(void);

extern const module_S wifi_module;
// Forward declaration of command handler
STATIC bool wifi_cmd_handler(const cmd_parsed_t *parsed);

const module_S wifi_module = {
    .module_name = "wifi",
    .module_init = wifi_init,
    .module_process_10Hz = wifi_process_10Hz,
    .module_cmd_handler = wifi_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s wifi_states[] = {
    [STATE_STARTUP] = {
        .process = NULL,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_INITIALIZATION] = {
        .process = NULL,
        .onEntry = wifi_state_initialization_on_entry,
        .onExit = NULL
    },
    [STATE_CONNECTING] = {
        .process = NULL,
        .onEntry = wifi_state_connecting_on_entry,
        .onExit = NULL
    },
    [STATE_ACTIVE] = {
        .process = wifi_state_active_process,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_FAULTED] = {
        .process = NULL,
        .onEntry = wifi_state_faulted_on_entry,
        .onExit = NULL
    }
};

STATIC state_machine_s wifi_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = wifi_transition_logic,
    .states = wifi_states
};

// Module state
STATIC wifi_state_machine_inputs_t wifi_state_machine_inputs = {0};
STATIC wifi_fault_code_e wifi_fault_code = FAULT_NONE;
STATIC wifi_config_t wifi_config = {
    .ssid = "YourSSID",           // TODO: Set via command or config
    .password = "YourPassword",   // TODO: Set via command or config
    .server_ip = "192.168.1.100", // TODO: Set via command or config
    .server_port = 5000           // TODO: Set via command or config
};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool send_at_command(const char *cmd, const char *expected_response, uint32_t timeout_ms)
{
    // TODO: Implement AT command send/receive
    // 1. Send command via UART
    // 2. Wait for response with timeout
    // 3. Parse response and check for expected string
    return false;
}

STATIC bool wifi_init_esp8266(void)
{
    // TODO: Send initialization AT commands
    // AT - Test connection
    // AT+RST - Reset module
    // AT+CWMODE=1 - Set to station mode
    // ATE0 - Disable echo
    return false;
}

STATIC bool wifi_connect_to_network(void)
{
    // TODO: Send WiFi connection command
    // AT+CWJAP="ssid","password"
    return false;
}

STATIC bool wifi_establish_tcp_connection(void)
{
    // TODO: Establish TCP connection to server
    // AT+CIPSTART="TCP","server_ip",port
    return false;
}

STATIC void wifi_send_telemetry_packet(void)
{
    // TODO: Get latest IMU data (need to add getter to IMU module)
    // TODO: Format packet (JSON, binary, or custom protocol)
    // TODO: Send via AT+CIPSEND command
    
    // Example packet format:
    // {"roll": 12.5, "pitch": -3.2, "yaw": 180.0, "timestamp": 12345}
}

STATIC void wifi_init(void)
{
    // TODO: Initialize ESP8266 UART hardware via platform layer
    // State machine is already initialized with static values
}

STATIC void wifi_process_10Hz(void)
{
    // Run state machine at 10Hz
    wifi_state_machine_sample_inputs();
    state_machine_periodic(&wifi_state_machine);
}

STATIC void wifi_state_machine_sample_inputs(void)
{
    wifi_state_machine_inputs.fault_present = (wifi_fault_code != FAULT_NONE);
    // TODO: Update other inputs based on actual status
}

STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition to INITIALIZATION after ESP8266 boot delay (30 ticks at 10Hz = 3000ms)
            if (stateTimer >= MS_TO_10HZ_TICKS(STARTUP_DELAY_MS)) {
                nextState = STATE_INITIALIZATION;
            }
            break;
            
        case STATE_INITIALIZATION:
            // Transition to CONNECTING when ESP is ready
            if (wifi_state_machine_inputs.esp_ready) {
                nextState = STATE_CONNECTING;
            }
            else if (wifi_state_machine_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;
            
        case STATE_CONNECTING:
            // Transition to ACTIVE when connected
            if (wifi_state_machine_inputs.tcp_connected) {
                nextState = STATE_ACTIVE;
            }
            else if (wifi_state_machine_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;
            
        case STATE_ACTIVE:
            // Stay active unless fault detected
            if (wifi_state_machine_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;
            
        case STATE_FAULTED:
            // TODO: Add retry logic - could transition back to INITIALIZATION
            // For now, stay in fault state
            break;
    }
    
    return nextState;
}

STATIC void wifi_state_initialization_on_entry(uint16_t prevState)
{
    uart_manager_printf("WiFi: Initializing ESP8266...\r\n");
    
    // Initialize ESP8266 module
    bool success = wifi_init_esp8266();
    
    if (success) {
        wifi_state_machine_inputs.esp_ready = true;
        wifi_fault_code = FAULT_NONE;
        uart_manager_printf("WiFi: ESP8266 ready\r\n");
    } else {
        wifi_fault_code = FAULT_ESP_NO_RESPONSE;
        uart_manager_printf("WiFi: ESP8266 init failed\r\n");
    }
}

STATIC void wifi_state_connecting_on_entry(uint16_t prevState)
{
    uart_manager_printf("WiFi: Connecting to %s...\r\n", wifi_config.ssid);
    
    // Connect to WiFi network
    bool wifi_success = wifi_connect_to_network();
    if (!wifi_success) {
        wifi_fault_code = FAULT_WIFI_CONNECT_FAILED;
        uart_manager_printf("WiFi: Network connection failed\r\n");
        return;
    }
    
    wifi_state_machine_inputs.wifi_connected = true;
    uart_manager_printf("WiFi: Connected to network\r\n");
    
    // Establish TCP connection
    bool tcp_success = wifi_establish_tcp_connection();
    if (!tcp_success) {
        wifi_fault_code = FAULT_TCP_CONNECT_FAILED;
        uart_manager_printf("WiFi: TCP connection failed\r\n");
        return;
    }
    
    wifi_state_machine_inputs.tcp_connected = true;
    uart_manager_printf("WiFi: TCP connected to %s:%d\r\n", 
                       wifi_config.server_ip, wifi_config.server_port);
}

STATIC void wifi_state_active_process(void)
{
    // Send telemetry packet every time this is called (10Hz)
    wifi_send_telemetry_packet();
}

STATIC void wifi_state_faulted_on_entry(uint16_t prevState)
{
    // Log fault code
    uart_manager_printf("WiFi: FAULT - Code %d\r\n", wifi_fault_code);
    
    // TODO: Implement fault recovery strategy
    // Could set a retry timer and transition back to INITIALIZATION
}

/*---------------------------------------------------------------------------
 * Command Handler Implementation
 *---------------------------------------------------------------------------*/
STATIC bool wifi_cmd_handler(const cmd_parsed_t *parsed)
{
    // TODO: Implement commands like:
    // wifi config <ssid> <password>
    // wifi connect
    // wifi status
    // wifi disconnect
    
    uart_manager_printf("WiFi command handler - not yet implemented\r\n");
    return false;
}
