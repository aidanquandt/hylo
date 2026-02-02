/*---------------------------------------------------------------------------
 * @file    wifi.c
 * @brief   WiFi telemetry module for IMU data transmission via ESP8266
 *---------------------------------------------------------------------------*/

// Set to 0 to disable WiFi module
#define WIFI_MODULE_ENABLED 1

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "wifi.h"
#include "imu.h"
#include "module.h"
#include "uart_manager.h"
#include "uart_cmd_router.h"
#include "platform_uart.h"
#include "state_machine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include "wifi_config.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STARTUP_DELAY_MS            (3000U)  // ESP8266 boot time
#define AT_COMMAND_TIMEOUT_MS       (2000U)  // AT command response timeout

// External UART handle (from platform layer)
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
#define WIFI_UART (&huart2)


/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum {
    STATE_STARTUP,
    STATE_SEND_RESET,
    STATE_WAIT_RESET,
    STATE_SEND_MODE,
    STATE_WAIT_MODE,
    STATE_SEND_ECHO_OFF,
    STATE_WAIT_ECHO_OFF,
    STATE_SEND_JOIN_WIFI,
    STATE_WAIT_JOIN_WIFI,
    STATE_SEND_TCP_CONNECT,
    STATE_WAIT_TCP_CONNECT,
    STATE_VERIFY_TCP,
    STATE_ACTIVE,
    STATE_FAULTED
} wifi_state_E;

typedef struct {
    bool command_sent;
    bool response_received;
    bool response_ok;
    bool fault_present;
    uint8_t retry_count;
} wifi_state_inputs_t;

typedef struct {
    uint8_t data[128];
    uint16_t len;
} wifi_payload_t;

/* ---------------------- Non-blocking RX token flags (used in send task) ------------------ */
typedef struct {
    volatile bool got_gt;        // '>' prompt
    volatile bool got_ok;        // "OK"
    volatile bool got_send_ok;   // "SEND OK"
    volatile bool got_error;     // "ERROR"
    volatile bool got_closed;    // "CLOSED"
} esp_rx_flags_t;

typedef enum {
    ESP_SEND_IDLE = 0,
    ESP_SEND_WAIT_GT,
    ESP_SEND_WAIT_OK
} esp_send_state_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void esp_rx_init(void);

/* Non-blocking helpers for CIPSEND (only used after init is complete) */
STATIC void esp_rx_flags_clear(void);
STATIC void esp_rx_pump_and_parse(void);
STATIC bool esp_tcp_send_start_nb(const uint8_t *payload, uint16_t len);
STATIC bool esp_tcp_send_process_nb(bool *done_ok);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void wifi_init(void);
STATIC void wifi_process_10Hz(void);

// State machine functions
STATIC void wifi_state_machine_sample_inputs(void);
STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void wifi_state_send_reset_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_reset_process(void);
STATIC void wifi_state_send_mode_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_mode_process(void);
STATIC void wifi_state_send_echo_off_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_echo_off_process(void);
STATIC void wifi_state_send_join_wifi_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_join_wifi_process(void);
STATIC void wifi_state_send_tcp_connect_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_tcp_connect_process(void);
STATIC void wifi_state_verify_tcp_on_entry(uint16_t prevState);
STATIC void wifi_state_verify_tcp_process(void);
STATIC void wifi_state_active_on_entry(uint16_t prevState);
STATIC void wifi_state_active_process(void);
STATIC void wifi_state_faulted_on_entry(uint16_t prevState);

// Non-blocking command helpers
STATIC void wifi_start_command(const char *cmd);
STATIC void wifi_check_response(const char *expected_token, uint32_t timeout_ms);

extern const module_S wifi_module;

#if WIFI_MODULE_ENABLED
const module_S wifi_module = {
    .module_name = "wifi",
    .module_init = wifi_init,
    .module_process_10Hz = wifi_process_10Hz,
};
#else
// WiFi module disabled
const module_S wifi_module = {
    .module_name = "wifi",
    .module_init = NULL,
    .module_process_10Hz = NULL,
};
#endif

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC const state_s wifi_states[] = {
    [STATE_STARTUP] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_RESET] = {.process = NULL, .onEntry = wifi_state_send_reset_on_entry, .onExit = NULL},
    [STATE_WAIT_RESET] = {.process = wifi_state_wait_reset_process, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_MODE] = {.process = NULL, .onEntry = wifi_state_send_mode_on_entry, .onExit = NULL},
    [STATE_WAIT_MODE] = {.process = wifi_state_wait_mode_process, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_ECHO_OFF] = {.process = NULL, .onEntry = wifi_state_send_echo_off_on_entry, .onExit = NULL},
    [STATE_WAIT_ECHO_OFF] = {.process = wifi_state_wait_echo_off_process, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_JOIN_WIFI] = {.process = NULL, .onEntry = wifi_state_send_join_wifi_on_entry, .onExit = NULL},
    [STATE_WAIT_JOIN_WIFI] = {.process = wifi_state_wait_join_wifi_process, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_TCP_CONNECT] = {.process = NULL, .onEntry = wifi_state_send_tcp_connect_on_entry, .onExit = NULL},
    [STATE_WAIT_TCP_CONNECT] = {.process = wifi_state_wait_tcp_connect_process, .onEntry = NULL, .onExit = NULL},
    [STATE_VERIFY_TCP] = {.process = wifi_state_verify_tcp_process, .onEntry = wifi_state_verify_tcp_on_entry, .onExit = NULL},
    [STATE_ACTIVE] = {.process = wifi_state_active_process, .onEntry = wifi_state_active_on_entry, .onExit = NULL},
    [STATE_FAULTED] = {.process = NULL, .onEntry = wifi_state_faulted_on_entry, .onExit = NULL}
};

STATIC state_machine_s wifi_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = wifi_transition_logic,
    .states = wifi_states
};

STATIC StreamBufferHandle_t rxStream = NULL;
STATIC QueueHandle_t txQueue = NULL;

// DMA buffer in D2 SRAM (non-cacheable region on STM32H7)
// Use section attribute to place in SRAM2/D2 domain
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
STATIC uint8_t rx_dma_buf[128] = {0};

STATIC wifi_state_inputs_t state_inputs = {0};

// Non-blocking command tracking
STATIC char cmd_response_window[512] = {0};
STATIC size_t cmd_response_used = 0;
STATIC TickType_t cmd_start_time = 0;

// Diagnostics
STATIC volatile uint32_t rx_callback_count = 0;
STATIC volatile uint32_t rx_total_bytes = 0;

/* ---------------------- Non-blocking send state -------------------------- */
STATIC esp_rx_flags_t rx_flags = {0};
STATIC char rx_line[256] = {0};
STATIC size_t rx_line_used = 0;

STATIC esp_send_state_t send_state = ESP_SEND_IDLE;
STATIC const uint8_t *send_payload = NULL;
STATIC uint16_t send_len = 0;
STATIC TickType_t send_deadline = 0;

// WiFi configuration
STATIC const char *SSID = WIFI_SSID;
STATIC const char *PASS = WIFI_PASS;
STATIC const char *HOST = WIFI_HOST;
STATIC const uint16_t PORT = WIFI_PORT;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void wifi_init(void)
{
    uart_manager_print("[WiFi] Module init started\r\n");

    // Create stream buffer for RX data (DMA ISR -> State machine)
    rxStream = xStreamBufferCreate(512, 1);
    configASSERT(rxStream != NULL);

    // Create queue for TX data (for future use)
    txQueue = xQueueCreate(20, sizeof(wifi_payload_t));
    configASSERT(txQueue != NULL);

    esp_rx_init();
}

STATIC void wifi_process_10Hz(void)
{
    // Only pump RX for non-blocking sends when in ACTIVE state
    if (wifi_state_machine.curr_state == STATE_ACTIVE) {
        esp_rx_pump_and_parse();
    }

    // Update state machine
    wifi_state_machine_sample_inputs();
    state_machine_periodic(&wifi_state_machine);
}

STATIC void wifi_state_machine_sample_inputs(void)
{
    // State inputs updated by init task and state handlers
}

STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    //static uint16_t last_printed_state = 0xFF;

    // Print state on first entry
    // if (currentState != last_printed_state) {
    //     const char* state_names[] = {"STARTUP", "SEND_RESET", "WAIT_RESET", "SEND_MODE", "WAIT_MODE", 
    //                                   "SEND_ECHO_OFF", "WAIT_ECHO_OFF", "SEND_JOIN_WIFI", "WAIT_JOIN_WIFI",
    //                                   "SEND_TCP_CONNECT", "WAIT_TCP_CONNECT", "VERIFY_TCP", "ACTIVE", "FAULTED"};
    //     if (currentState < 14) {
    //         uart_manager_print("[WiFi] State: %s\r\n", state_names[currentState]);
    //     }
    //     last_printed_state = currentState;
    // }

    switch (currentState) {
        case STATE_STARTUP:
            if (stateTimer >= MS_TO_10HZ_TICKS(STARTUP_DELAY_MS)) {
                nextState = STATE_SEND_RESET;
            }
            break;

        case STATE_SEND_RESET:
            nextState = STATE_WAIT_RESET;
            break;

        case STATE_WAIT_RESET:
            if (state_inputs.response_ok) {
                nextState = STATE_SEND_MODE;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_SEND_MODE:
            nextState = STATE_WAIT_MODE;
            break;

        case STATE_WAIT_MODE:
            if (state_inputs.response_ok) {
                nextState = STATE_SEND_ECHO_OFF;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_SEND_ECHO_OFF:
            nextState = STATE_WAIT_ECHO_OFF;
            break;

        case STATE_WAIT_ECHO_OFF:
            if (state_inputs.response_ok) {
                nextState = STATE_SEND_JOIN_WIFI;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_SEND_JOIN_WIFI:
            nextState = STATE_WAIT_JOIN_WIFI;
            break;

        case STATE_WAIT_JOIN_WIFI:
            if (state_inputs.response_ok) {
                nextState = STATE_SEND_TCP_CONNECT;
            } else if (state_inputs.fault_present) {
                // Retry up to 3 times
                if (state_inputs.retry_count < 3) {
                    state_inputs.retry_count++;
                    uart_manager_print("[WiFi] Join retry %d/3\r\n", state_inputs.retry_count);
                    nextState = STATE_SEND_JOIN_WIFI;
                } else {
                    nextState = STATE_FAULTED;
                }
            }
            break;

        case STATE_SEND_TCP_CONNECT:
            nextState = STATE_WAIT_TCP_CONNECT;
            break;

        case STATE_WAIT_TCP_CONNECT:
            if (state_inputs.response_ok) {
                nextState = STATE_VERIFY_TCP;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_VERIFY_TCP:
            if (state_inputs.response_ok) {
                nextState = STATE_ACTIVE;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            } else if (state_inputs.response_received) {
                // response_received + !response_ok + !fault_present => retry path
                nextState = STATE_SEND_TCP_CONNECT;
            }
            break;

        case STATE_ACTIVE:
            if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_FAULTED:
            break;
    }

    return nextState;
}

STATIC void wifi_state_send_reset_on_entry(uint16_t prevState)
{
    (void)prevState;
    wifi_start_command("AT+RST\r\n");
}

STATIC void wifi_state_wait_reset_process(void)
{
    wifi_check_response("ready", 5000);
}

STATIC void wifi_state_send_mode_on_entry(uint16_t prevState)
{
    (void)prevState;
    wifi_start_command("AT+CWMODE=1\r\n");
}

STATIC void wifi_state_wait_mode_process(void)
{
    wifi_check_response("OK", 2000);
}

STATIC void wifi_state_send_echo_off_on_entry(uint16_t prevState)
{
    (void)prevState;
    wifi_start_command("ATE0\r\n");
}

STATIC void wifi_state_wait_echo_off_process(void)
{
    wifi_check_response("OK", 2000);
}

STATIC void wifi_state_send_join_wifi_on_entry(uint16_t prevState)
{
    (void)prevState;
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASS);
    wifi_start_command(cmd);
}

STATIC void wifi_state_wait_join_wifi_process(void)
{
    // Keep collecting responses
    wifi_check_response("WIFI GOT IP", 20000);

    // If we already succeeded, don't let later DISCONNECT in the same window ruin it
    if (state_inputs.response_ok) {
        return;
    }

    // During CWJAP, "WIFI DISCONNECT" happens sometimes before we connect again.
    // Only treat it as a failure if we also see FAIL/ERROR or we timeout.
    if (strstr(cmd_response_window, "FAIL") || strstr(cmd_response_window, "ERROR")) {
        uart_manager_print("[WiFi] Join failed (FAIL/ERROR)\r\n");
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        state_inputs.command_sent = false;
        return;
    }
}

STATIC void wifi_state_send_tcp_connect_on_entry(uint16_t prevState)
{
    (void)prevState;
    // Clear state inputs to prevent false transitions
    state_inputs.command_sent = false;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;
}

STATIC void wifi_state_wait_tcp_connect_process(void)
{
    static bool checking_started = false;

    const uint32_t timeout_ms = 20000;

    // Wait 1 second after GOT IP before sending CIPSTART
    if (wifi_state_machine.timer < MS_TO_10HZ_TICKS(1000)) {
        return;
    }

    // Send command once after delay
    if (!state_inputs.command_sent) {
        char cmd[256];
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n", HOST, (unsigned)PORT);
        wifi_start_command(cmd);
        return;
    }

    if (!checking_started) {
        uart_manager_print("[WiFi] Starting to check for CONNECT response...\r\n");
        checking_started = true;
    }

    if (!state_inputs.command_sent || state_inputs.response_received) {
        checking_started = false;
        return;
    }

    // Timeout
    if ((xTaskGetTickCount() - cmd_start_time) > pdMS_TO_TICKS(timeout_ms)) {
        uart_manager_print("[WiFi] CIPSTART TIMEOUT after %lu ms\r\n", (unsigned long)timeout_ms);
        uart_manager_print("[WiFi] Final window: '%s'\r\n", cmd_response_window);

        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;   // hard fault: nothing useful came back
        checking_started = false;
        return;
    }

    // Drain RX into rolling window
    uint8_t tmp[256];
    size_t n;

    while ((n = xStreamBufferReceive(rxStream, tmp, sizeof(tmp), 0)) > 0) {

        // Make room
        size_t free = (sizeof(cmd_response_window) - 1) - cmd_response_used;
        if (n > free) {
            size_t drop = n - free;
            if (drop >= cmd_response_used) {
                cmd_response_used = 0;
                cmd_response_window[0] = 0;
            } else {
                memmove(cmd_response_window,
                        cmd_response_window + drop,
                        cmd_response_used - drop);
                cmd_response_used -= drop;
            }
        }

        memcpy(cmd_response_window + cmd_response_used, tmp, n);
        cmd_response_used += n;
        cmd_response_window[cmd_response_used] = 0;
    }

    // If ESP says it's busy, extend the deadline (do NOT advance state yet)
    if (strstr(cmd_response_window, "busy p")) {
        cmd_start_time = xTaskGetTickCount(); // extend wait
        return;
    }

    // Fail fast tokens
    if (strstr(cmd_response_window, "DNS FAIL") ||
        strstr(cmd_response_window, "FAIL") ||
        strstr(cmd_response_window, "ERROR") ||
        strstr(cmd_response_window, "CLOSED") ||
        strstr(cmd_response_window, "LINK IS NOT VALID")) {

        uart_manager_print("[WiFi] CIPSTART FAILED\r\n");
        uart_manager_print("[WiFi] Final window: '%s'\r\n", cmd_response_window);

        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        checking_started = false;
        return;
    }

    // Success ONLY if CONNECT (or already connected)
    if (strstr(cmd_response_window, "ALREADY CONNECTED") ||
        strstr(cmd_response_window, "CONNECT")) {

        uart_manager_print("[WiFi] CIPSTART CONNECTED\r\n");
        uart_manager_print("[WiFi] Final window: '%s'\r\n", cmd_response_window);

        state_inputs.response_received = true;
        state_inputs.response_ok = true;
        state_inputs.fault_present = false;
        state_inputs.command_sent = false;
        checking_started = false;
        return;
    }

    // Otherwise keep waiting (do nothing)
}

STATIC void wifi_state_verify_tcp_on_entry(uint16_t prevState)
{
    (void)prevState;
    // Clear previous state flags
    state_inputs.command_sent = false;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;
}

STATIC void wifi_state_verify_tcp_process(void)
{
    // Wait 1.5 seconds after CONNECT before checking status
    if (wifi_state_machine.timer < MS_TO_10HZ_TICKS(1500)) {
        return;
    }

    if (!state_inputs.command_sent) {
        // Drain old RX
        uint8_t dump[256];
        while (xStreamBufferReceive(rxStream, dump, sizeof(dump), 0) > 0) {}

        uart_manager_print("[WiFi] Sending CIPSTATUS...\r\n");
        wifi_start_command("AT+CIPSTATUS\r\n");
        return;
    }

    // Collect response looking for STATUS:3
    wifi_check_response("STATUS:", 2000);

    if (!state_inputs.response_received) {
        return;
    }

    // We got some STATUS response in cmd_response_window now.
    if (strstr(cmd_response_window, "STATUS:3")) {
        uart_manager_print("[WiFi] STATUS:3 (TCP connected)\r\n");
        state_inputs.response_ok = true;
        state_inputs.fault_present = false;
        state_inputs.command_sent = false;
        return;
    }

    if (strstr(cmd_response_window, "STATUS:2")) {
        uart_manager_print("[WiFi] STATUS:2 (no TCP) -> retry CIPSTART\r\n");

        // Not a hard fault; trigger retry path
        state_inputs.response_ok = false;
        state_inputs.fault_present = false;     // <-- important
        state_inputs.command_sent = false;
        return;
    }

    // Anything else treat as fault
    uart_manager_print("[WiFi] Unexpected CIPSTATUS: '%s'\r\n", cmd_response_window);
    state_inputs.response_ok = false;
    state_inputs.fault_present = true;
    state_inputs.command_sent = false;
}

STATIC void wifi_state_active_on_entry(uint16_t prevState)
{
    (void)prevState;
    
    // Drain any leftover RX data from initialization
    uint8_t dump[256];
    while (xStreamBufferReceive(rxStream, dump, sizeof(dump), 0) > 0) {}
    
    // Clear rx flags and line buffer
    esp_rx_flags_clear();
    rx_line_used = 0;
    rx_line[0] = 0;
}

STATIC void wifi_state_active_process(void)
{
    // Start non-blocking send
    const char *msg = "Hello\r\n";
    if (send_state == ESP_SEND_IDLE) {
        esp_tcp_send_start_nb((const uint8_t*)msg, (uint16_t)strlen(msg));
    }

    // Process ongoing send
    if (send_state != ESP_SEND_IDLE) {
        bool done_ok = false;
        bool finished = esp_tcp_send_process_nb(&done_ok);
        if (finished && !done_ok) {
            uart_manager_print("[WiFi] Send FAILED\r\n");
        }
    }
}

STATIC void wifi_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;
    uart_manager_print("[WiFi] WiFi module FAULTED\r\n");
}

// Non-blocking command helpers
STATIC void wifi_start_command(const char *cmd)
{
    // Drain old RX data
    uint8_t dump[256];
    while (xStreamBufferReceive(rxStream, dump, sizeof(dump), 0) > 0) {}

    // Send command - use blocking for initialization commands
    // (wifi module runs in periodic callback, not dedicated task)
    esp_uart_transmit_blocking((uint8_t*)cmd, (uint16_t)strlen(cmd));

    // Reset tracking
    cmd_response_window[0] = 0;
    cmd_response_used = 0;
    cmd_start_time = xTaskGetTickCount();
    state_inputs.command_sent = true;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;
}

STATIC void wifi_check_response(const char *expected_token, uint32_t timeout_ms)
{
    if (!state_inputs.command_sent || state_inputs.response_received) {
        return;
    }

    if ((xTaskGetTickCount() - cmd_start_time) > pdMS_TO_TICKS(timeout_ms)) {
        uart_manager_print("[WiFi] Timeout waiting for: %s\r\n", expected_token);
        uart_manager_print("[WiFi] Received: '%s'\r\n", cmd_response_window);
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }

    uint8_t tmp[256];
    size_t n;

    // Drain ALL available bytes each call
    while ((n = xStreamBufferReceive(rxStream, tmp, sizeof(tmp), 0)) > 0) {

        // Make room in rolling window if needed
        size_t free = (sizeof(cmd_response_window) - 1) - cmd_response_used;
        if (n > free) {
            size_t drop = n - free;

            // Clamp drop to avoid underflow/corruption
            if (drop >= cmd_response_used) {
                cmd_response_used = 0;
                cmd_response_window[0] = 0;
            } else {
                memmove(cmd_response_window,
                        cmd_response_window + drop,
                        cmd_response_used - drop);
                cmd_response_used -= drop;
            }
        }

        memcpy(cmd_response_window + cmd_response_used, tmp, n);
        cmd_response_used += n;
        cmd_response_window[cmd_response_used] = 0;
    }

    // No new data? keep waiting
    // (note: we drained in a loop; if none was available, window unchanged)
    if (strstr(cmd_response_window, "busy p")) {
        cmd_start_time = xTaskGetTickCount();
        return;
    }

    if (strstr(cmd_response_window, expected_token)) {
        uart_manager_print("[WiFi] Found: %s\r\n", expected_token);
        state_inputs.response_received = true;
        state_inputs.response_ok = true;
        state_inputs.command_sent = false;
        return;
    }

    if (strstr(cmd_response_window, "ERROR")) {
        uart_manager_print("[WiFi] Received ERROR\r\n");
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }
}

STATIC void esp_rx_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/* ---------------------- Non-blocking RX parsing (send task only) ---------------------- */
STATIC void esp_rx_flags_clear(void)
{
    memset((void*)&rx_flags, 0, sizeof(rx_flags));
}

STATIC void esp_rx_pump_and_parse(void)
{
    uint8_t tmp[256];
    size_t n;

    // Drain all currently available bytes and update flags.
    while ((n = xStreamBufferReceive(rxStream, tmp, sizeof(tmp), 0)) > 0) {
        // Debug: print received bytes
        //uart_manager_print("[WiFi RX] %.*s", (int)n, tmp);
        
        for (size_t i = 0; i < n; i++) {
            char c = (char)tmp[i];

            // '>' prompt may appear without CRLF
            if (c == '>') {
                rx_flags.got_gt = true;
            }

            // Accumulate for line parsing (CRLF-terminated responses)
            if (rx_line_used < sizeof(rx_line) - 1) {
                rx_line[rx_line_used++] = c;
                rx_line[rx_line_used] = 0;
            } else {
                // overflow: reset accumulator
                rx_line_used = 0;
                rx_line[0] = 0;
            }

            if (c == '\n') {
                // classify line tokens
                if (strstr(rx_line, "SEND OK")) rx_flags.got_send_ok = true;
                if (strstr(rx_line, "OK"))      rx_flags.got_ok = true;
                if (strstr(rx_line, "ERROR"))   rx_flags.got_error = true;
                if (strstr(rx_line, "CLOSED"))  rx_flags.got_closed = true;

                // reset for next line
                rx_line_used = 0;
                rx_line[0] = 0;
            }
        }
    }
}

/* ---------------------- Non-blocking CIPSEND (send task only) ------------------------- */
STATIC bool esp_tcp_send_start_nb(const uint8_t *payload, uint16_t len)
{
    if (send_state != ESP_SEND_IDLE) {
        return false; // busy
    }

    uart_manager_print("[WiFi] Starting send: %u bytes\r\n", (unsigned)len);
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u\r\n", (unsigned)len);

    // Scope tokens to this transaction
    esp_rx_flags_clear();
    rx_line_used = 0;
    rx_line[0] = 0;

    // Use blocking transmit for command to ensure it completes before checking response
    esp_uart_transmit_blocking((uint8_t*)cmd, (uint16_t)strlen(cmd));

    send_payload = payload;
    send_len = len;
    send_state = ESP_SEND_WAIT_GT;
    send_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
    return true;
}

// Returns true when finished; *done_ok indicates success/fail.
STATIC bool esp_tcp_send_process_nb(bool *done_ok)
{
    if (done_ok) *done_ok = false;

    // Parse any new bytes
    esp_rx_pump_and_parse();

    // fail-fast tokens
    if (rx_flags.got_error || rx_flags.got_closed) {
        uart_manager_print("[WiFi] Send error/closed\r\n");
        send_state = ESP_SEND_IDLE;
        return true;
    }

    // timeout
    if ((int32_t)(xTaskGetTickCount() - send_deadline) > 0) {
        uart_manager_print("[WiFi] Send timeout in state %d\r\n", send_state);
        send_state = ESP_SEND_IDLE;
        return true;
    }

    switch (send_state) {
        case ESP_SEND_WAIT_GT:
            if (!rx_flags.got_gt) {
                return false; // keep waiting (non-blocking)
            }

            uart_manager_print("[WiFi] Got >, sending payload\r\n");
            // got prompt: send payload once - use blocking to ensure completion
            esp_uart_transmit_blocking((uint8_t*)send_payload, send_len);

            // now wait for SEND OK / OK
            send_state = ESP_SEND_WAIT_OK;
            send_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
            return false;

        case ESP_SEND_WAIT_OK:
            // some firmwares only give "SEND OK", some also include "OK"
            if (rx_flags.got_send_ok || rx_flags.got_ok) {
                send_state = ESP_SEND_IDLE;
                if (done_ok) *done_ok = true;
                return true;
            }
            return false;

        default:
            send_state = ESP_SEND_IDLE;
            return true;
    }
}


/*---------------------------------------------------------------------------
 * HAL Callback Implementations
 *---------------------------------------------------------------------------*/

void ESP_RX_UARTEx_RXEventCallback(uint16_t size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Push bytes into stream buffer (DMA → Task)
    xStreamBufferSendFromISR(rxStream, rx_dma_buf, size, &xHigherPriorityTaskWoken);

    // Re-arm DMA reception
    HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void ESP_RX_UART_ErrorCallback(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/*---------------------------------------------------------------------------
 * Command Handler Implementation
 *---------------------------------------------------------------------------*/
