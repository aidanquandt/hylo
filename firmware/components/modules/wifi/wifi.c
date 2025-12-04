/*---------------------------------------------------------------------------
 * @file    wifi.c
 * @brief   ESP01 WiFi module interface
 * @details Manages ESP01 WiFi module via UART7 with DMA circular RX buffer
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "wifi.h"
#include "module.h"
#include "platform_uart.h"
#include "platform_gpio.h"
#include "uart_manager.h"
#include "state_machine.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define WIFI_TASK_STACK_SIZE        1024U           // Stack size in words
#define WIFI_TASK_PRIORITY          osPriorityNormal

#define ESP_RX_BUFFER_SIZE          512U            // DMA circular buffer size
#define ESP_CMD_TIMEOUT_MS          5000U           // Command response timeout

// WiFi configuration - see wifi_config.h (not tracked in git)
#include "wifi_config.h"

// State machine states
typedef enum {
    STATE_STARTUP,              // Initial delay after power-up
    STATE_TEST_AT,              // Test AT communication
    STATE_SET_MODE,             // Set WiFi station mode
    STATE_CONNECT_WIFI,         // Connect to WiFi network
    STATE_CONNECT_TCP,          // Connect to TCP server
    STATE_CONNECTED,            // Connected and ready to send
    STATE_ERROR                 // Error state
} wifi_state_E;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void wifi_init(void);
STATIC void wifi_process_10Hz(void);

extern const module_S wifi_module;
const module_S wifi_module = {
    .module_init = wifi_init,
    .module_process_10Hz = wifi_process_10Hz,
};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
// State machine
STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void wifi_state_startup(void);
STATIC void wifi_state_test_AT(void);
STATIC void wifi_state_set_mode(void);
STATIC void wifi_state_connect_wifi(void);
STATIC void wifi_state_connect_tcp(void);
STATIC void wifi_state_error(void);
STATIC void wifi_send_data(void);

// Helper functions
STATIC bool esp_send_command(const char *cmd);
STATIC bool esp_check_response(const char *expected);
STATIC uint16_t esp_get_available_bytes(void);
STATIC bool esp_read_response(char *buffer, uint16_t max_len);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s wifi_states[] = {
    [STATE_STARTUP] = {
        .process = wifi_state_startup,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_TEST_AT] = {
        .process = wifi_state_test_AT,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_SET_MODE] = {
        .process = wifi_state_set_mode,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_CONNECT_WIFI] = {
        .process = wifi_state_connect_wifi,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_CONNECT_TCP] = {
        .process = wifi_state_connect_tcp,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_ERROR] = {
        .process = wifi_state_error,
        .onEntry = NULL,
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

// DMA circular buffer for ESP responses
STATIC uint8_t esp_rx_buffer[ESP_RX_BUFFER_SIZE];
STATIC uint16_t esp_rx_read_pos = 0;

// State tracking
STATIC bool wifi_ready = false;
STATIC bool wifi_connected = false;
STATIC bool dma_started = false;

// Statistics
STATIC uint32_t tx_count = 0;
STATIC uint32_t rx_count = 0;
STATIC uint32_t error_count = 0;
STATIC uint32_t retry_count = 0;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Send AT command to ESP01
 * @param cmd Command string (must include \r\n)
 * @return true if sent successfully, false otherwise
 */
STATIC bool esp_send_command(const char *cmd)
{
    if (cmd == NULL) {
        return false;
    }
    
    size_t len = strlen(cmd);
    platform_uart_status_E status = platform_uart_esp_transmit((const uint8_t *)cmd, len);
    
    if (status == PLATFORM_UART_SUCCESS) {
        uart_manager_print("[WiFi] TX: %s", cmd);
        return true;
    }
    
    return false;
}

/**
 * @brief Get number of bytes available in DMA circular buffer
 * @return Number of unread bytes
 */
STATIC uint16_t esp_get_available_bytes(void)
{
    // Get current DMA counter via platform layer
    uint16_t dma_counter = platform_uart_esp_get_dma_counter();
    
    // Calculate write position (buffer_size - remaining)
    uint16_t write_pos = ESP_RX_BUFFER_SIZE - dma_counter;
    
    // Calculate available bytes
    if (write_pos >= esp_rx_read_pos) {
        return write_pos - esp_rx_read_pos;
    } else {
        // Wrapped around
        return (ESP_RX_BUFFER_SIZE - esp_rx_read_pos) + write_pos;
    }
}

/**
 * @brief Read response from ESP DMA circular buffer (non-blocking)
 * @param buffer Output buffer for response
 * @param max_len Maximum buffer length
 * @return true if data read, false if no data available
 */
STATIC bool esp_read_response(char *buffer, uint16_t max_len)
{
    if (buffer == NULL || max_len == 0) {
        return false;
    }
    
    uint16_t available = esp_get_available_bytes();
    
    if (available == 0) {
        return false;
    }
    
    uint16_t bytes_read = 0;
    
    // Read available bytes (up to max_len - 1 for null terminator)
    while (available > 0 && bytes_read < (max_len - 1)) {
        buffer[bytes_read++] = esp_rx_buffer[esp_rx_read_pos];
        esp_rx_read_pos = (esp_rx_read_pos + 1) % ESP_RX_BUFFER_SIZE;
        available--;
    }
    
    buffer[bytes_read] = '\0';
    
    if (bytes_read > 0) {
        rx_count++;
        uart_manager_print("[WiFi] RX: %s", buffer);
        return true;
    }
    
    return false;
}

/**
 * @brief Check if expected response received (non-blocking)
 * @param expected Expected substring in response
 * @return true if expected response found, false otherwise
 */
STATIC bool esp_check_response(const char *expected)
{
    char response[256];
    
    if (esp_read_response(response, sizeof(response))) {
        if (strstr(response, expected) != NULL) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief State machine transition logic
 * @param currentState Current state
 * @param stateTimer Time spent in current state (in 10Hz ticks)
 * @return Next state
 */
STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    const uint32_t MAX_RETRIES = 3;
    
    // Get elapsed time in milliseconds (stateTimer is in 100ms units at 10Hz)
    uint32_t elapsed_ms = stateTimer * 100;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition after 2 second delay
            if (elapsed_ms >= 2000) {
                nextState = STATE_TEST_AT;
                retry_count = 0;
            }
            break;
            
        case STATE_TEST_AT:
            // Check for timeout or success
            if (esp_check_response("OK")) {
                uart_manager_print("[WiFi] AT communication OK\n");
                nextState = STATE_SET_MODE;
                retry_count = 0;
            } else if (elapsed_ms >= 1000) {
                retry_count++;
                if (retry_count >= MAX_RETRIES) {
                    uart_manager_print("[WiFi] AT test failed\n");
                    nextState = STATE_ERROR;
                } else {
                    nextState = STATE_TEST_AT;  // Retry
                }
            }
            break;
            
        case STATE_SET_MODE:
            if (esp_check_response("OK")) {
                uart_manager_print("[WiFi] Station mode set\n");
                nextState = STATE_CONNECT_WIFI;
                retry_count = 0;
            } else if (elapsed_ms >= 2000) {
                retry_count++;
                if (retry_count >= MAX_RETRIES) {
                    nextState = STATE_ERROR;
                } else {
                    nextState = STATE_SET_MODE;  // Retry
                }
            }
            break;
            
        case STATE_CONNECT_WIFI:
            if (esp_check_response("OK")) {
                uart_manager_print("[WiFi] Connected to WiFi\n");
                nextState = STATE_CONNECT_TCP;
                retry_count = 0;
            } else if (elapsed_ms >= 10000) {
                retry_count++;
                if (retry_count >= MAX_RETRIES) {
                    uart_manager_print("[WiFi] WiFi connection failed\n");
                    nextState = STATE_ERROR;
                } else {
                    nextState = STATE_CONNECT_WIFI;  // Retry
                }
            }
            break;
            
        case STATE_CONNECT_TCP:
            if (esp_check_response("OK")) {
                uart_manager_print("[WiFi] TCP connected\n");
                nextState = STATE_CONNECTED;
                wifi_connected = true;
                wifi_ready = true;
                retry_count = 0;
            } else if (elapsed_ms >= 5000) {
                retry_count++;
                if (retry_count >= MAX_RETRIES) {
                    uart_manager_print("[WiFi] TCP connection failed\n");
                    nextState = STATE_ERROR;
                } else {
                    nextState = STATE_CONNECT_TCP;  // Retry
                }
            }
            break;
            
        case STATE_CONNECTED:
            // Send message every 5 seconds
            if (elapsed_ms >= 5000) {
                nextState = STATE_CONNECTED;  // Re-enter to send again
            }
            break;
            
        case STATE_ERROR:
            // Retry after 10 seconds
            if (elapsed_ms >= 10000) {
                uart_manager_print("[WiFi] Retrying...\n");
                error_count++;
                wifi_connected = false;
                wifi_ready = false;
                nextState = STATE_STARTUP;
            }
            break;
            
        default:
            nextState = STATE_STARTUP;
            break;
    }
    
    return nextState;
}

STATIC void wifi_state_startup(void){
    // Nothing to do - timer automatically increments
    uart_manager_print("[WiFi] Waiting for ESP boot...\n");
}

STATIC void wifi_state_test_AT(void){
    uart_manager_print("[WiFi] Testing AT...\n");
    esp_send_command("AT\r\n");
}

STATIC void wifi_state_set_mode(void){
    uart_manager_print("[WiFi] Setting station mode...\n");
    esp_send_command("AT+CWMODE=1\r\n");
}

STATIC void wifi_state_connect_wifi(void){
    char buffer[256];
    uart_manager_print("[WiFi] Connecting to WiFi...\n");
    snprintf(buffer, sizeof(buffer), "AT+CWJAP=\"%s\",\"%s\"\r\n", 
                WIFI_SSID, WIFI_PASSWORD);
    esp_send_command(buffer);
}

STATIC void wifi_state_connect_tcp(void){
    char buffer[256];
    uart_manager_print("[WiFi] Connecting to TCP...\n");
    snprintf(buffer, sizeof(buffer), "AT+CIPSTART=\"TCP\",\"%s\",%s\r\n", 
                TCP_SERVER_IP, TCP_SERVER_PORT);
    esp_send_command(buffer);
}

STATIC void wifi_state_error(void){
    uart_manager_print("[WiFi] Error state\n");
}

STATIC void wifi_send_data(void){
    if (!wifi_connected) {
        return;  // Not connected, skip sending
    }
    
    const char *message = "hello from stm32";
    size_t msg_len = strlen(message);
    char cmd_buffer[64];
    
    // Step 1: Tell ESP01 we want to send data (AT+CIPSEND=<length>)
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CIPSEND=%u\r\n", (unsigned int)msg_len);
    esp_send_command(cmd_buffer);
    
    // Small delay to wait for ">" prompt (ESP ready to receive data)
    osDelay(50);  // 50ms delay
    
    // Step 2: Send the actual message
    platform_uart_esp_transmit((const uint8_t *)message, msg_len);
    tx_count++;
    
    uart_manager_print("[WiFi] Sent: %s\n", message);
}


/**
 * @brief Initialize WiFi module
 */
STATIC void wifi_init(void)
{
    
    // Reset state
    wifi_ready = false;
    wifi_connected = false;
    dma_started = false;
    tx_count = 0;
    rx_count = 0;
    error_count = 0;
    retry_count = 0;
    esp_rx_read_pos = 0;
    
    // Start DMA reception
    if (platform_uart_esp_receive_dma_start(esp_rx_buffer, ESP_RX_BUFFER_SIZE) == PLATFORM_UART_SUCCESS) {
        dma_started = true;
        uart_manager_print("[WiFi] DMA started\n");
    }

    //wifi initialization done in state machine
    state_machine_periodic(&wifi_state_machine);
}

/**
 * @brief Process WiFi module at 10Hz
 */
STATIC void wifi_process_10Hz(void)
{
    //send hello from wifi module every 10 seconds if connected
    wifi_send_data();
    
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool wifi_is_connected(void)
{
    return wifi_connected;
}

uint32_t wifi_get_tx_count(void)
{
    return tx_count;
}

uint32_t wifi_get_rx_count(void)
{
    return rx_count;
}

bool wifi_is_ready(void)
{
    return wifi_ready;
}