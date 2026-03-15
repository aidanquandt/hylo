/*---------------------------------------------------------------------------
 * @file    wifi.c
 * @brief   WiFi telemetry module for data transmission via ESP8266
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "wifi.h"
#include "wifi_config.h"
#include "imu.h"
#include "module.h"
#include "uart_driver.h"
#include "state_machine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"
#include "common.h"
#include "feature_config.h"
#include "wifi_ota_parser.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STARTUP_DELAY_MS            (4000U)  // ESP8266 boot time
#define AT_COMMAND_TIMEOUT_MS       (2000U)  // AT command response timeout

STATIC inline StreamBufferHandle_t wifi_rx_stream(void)
{
    return uart_driver_get_rx_stream(UART_WIFI);
}

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
    STATE_SEND_CIPMODE,
    STATE_WAIT_CIPMODE,
    STATE_SEND_CIPSEND,
    STATE_WAIT_CIPSEND,
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

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
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
STATIC void wifi_state_send_cipmode_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_cipmode_process(void);
STATIC void wifi_state_send_cipsend_on_entry(uint16_t prevState);
STATIC void wifi_state_wait_cipsend_process(void);
STATIC void wifi_state_active_on_entry(uint16_t prevState);
STATIC void wifi_state_active_process(void);
STATIC void wifi_state_faulted_on_entry(uint16_t prevState);

// Non-blocking command helpers
STATIC void wifi_start_command(const char *cmd);
STATIC void wifi_check_response(const char *expected_token, uint32_t timeout_ms);

// Telemetry transmission
STATIC void wifi_transmit_telemetry_queue(void);


/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
#if FEATURE_WIFI_MODULE
STATIC void wifi_init(void);
STATIC void wifi_process_10Hz(void);

const module_S wifi_module= {
    .module_name         = "wifi",
    .module_init         = wifi_init,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = wifi_process_10Hz,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};
#else
const module_S wifi_module= {
    .module_name         = "wifi",
    .module_init         = NULL,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
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
    [STATE_SEND_CIPMODE] = {.process = NULL, .onEntry = wifi_state_send_cipmode_on_entry, .onExit = NULL},
    [STATE_WAIT_CIPMODE] = {.process = wifi_state_wait_cipmode_process, .onEntry = NULL, .onExit = NULL},
    [STATE_SEND_CIPSEND] = {.process = NULL, .onEntry = wifi_state_send_cipsend_on_entry, .onExit = NULL},
    [STATE_WAIT_CIPSEND] = {.process = wifi_state_wait_cipsend_process, .onEntry = NULL, .onExit = NULL},
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

STATIC wifi_state_inputs_t state_inputs = {0};

// Non-blocking command tracking
STATIC char cmd_response_window[512] = {0};
STATIC size_t cmd_response_used = 0;
STATIC TickType_t cmd_start_time = 0;

// TCP connection monitoring
STATIC volatile bool tcp_closed_detected = false;

// WiFi configuration
STATIC const char *SSID = WIFI_SSID;
STATIC const char *PASS = WIFI_PASS;
STATIC const char *HOST = WIFI_HOST;
STATIC const uint16_t PORT = WIFI_PORT;

// Telemetry queue
#define TELEMETRY_QUEUE_SIZE 16U
STATIC QueueHandle_t telemetry_queue = NULL;
STATIC struct
{
    volatile uint32_t events_pushed;
    volatile uint32_t events_dropped;
    volatile uint32_t events_transmitted;
    volatile uint32_t sequence;
} telemetry_stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

#if FEATURE_WIFI_MODULE
STATIC void wifi_init(void)
{
    telemetry_queue = xQueueCreate(TELEMETRY_QUEUE_SIZE, sizeof(telemetry_event_t));
    configASSERT(telemetry_queue != NULL);

    telemetry_stats.events_pushed = 0;
    telemetry_stats.events_dropped = 0;
    telemetry_stats.events_transmitted = 0;
    telemetry_stats.sequence = 0;

    ota_parser_init();
    uart_driver_rx_start(UART_WIFI);
}

STATIC void wifi_process_10Hz(void)
{
    state_machine_periodic(&wifi_state_machine);
}
#endif /* FEATURE_WIFI_MODULE */

STATIC uint16_t wifi_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    static uint16_t last_printed_state = 0xFF;

    /* Track state for first-entry / change detection when FEATURE_WIFI_PRINT_STATE is used */
    if (FEATURE_WIFI_PRINT_STATE && (currentState != last_printed_state)) {
        
        last_printed_state = currentState;
    }

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
                nextState = STATE_SEND_CIPMODE;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            } else if (state_inputs.response_received) {
                // response_received + !response_ok + !fault_present => retry path
                nextState = STATE_SEND_TCP_CONNECT;
            }
            break;

        case STATE_SEND_CIPMODE:
            nextState = STATE_WAIT_CIPMODE;
            break;

        case STATE_WAIT_CIPMODE:
            if (state_inputs.response_ok) {
                nextState = STATE_SEND_CIPSEND;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_SEND_CIPSEND:
            nextState = STATE_WAIT_CIPSEND;
            break;

        case STATE_WAIT_CIPSEND:
            if (state_inputs.response_ok) {
                nextState = STATE_ACTIVE;
            } else if (state_inputs.fault_present) {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_ACTIVE:
            if (tcp_closed_detected) {
                tcp_closed_detected = false;
                nextState = STATE_SEND_TCP_CONNECT;
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
    // Clear parser state
    state_inputs.command_sent = false;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;

    // FIX: clear old response window. this did nothing idk
    // cmd_response_used = 0;
    // cmd_response_window[0] = 0;
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
        checking_started = true;
    }

    if (!state_inputs.command_sent || state_inputs.response_received) {
        checking_started = false;
        return;
    }

    // Timeout
    if ((xTaskGetTickCount() - cmd_start_time) > pdMS_TO_TICKS(timeout_ms)) {

        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;   // hard fault: nothing useful came back
        checking_started = false;
        return;
    }

    // Drain RX into rolling window
    uint8_t tmp[256];
    size_t n;

    while ((n = xStreamBufferReceive(wifi_rx_stream(), tmp, sizeof(tmp), 0)) > 0) {

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


        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        checking_started = false;
        return;
    }

    // Success ONLY if CONNECT (or already connected)
    if (strstr(cmd_response_window, "ALREADY CONNECTED") ||
        strstr(cmd_response_window, "CONNECT")) {


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
        while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}

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
        state_inputs.response_ok = true;
        state_inputs.fault_present = false;
        state_inputs.command_sent = false;
        return;
    }

    if (strstr(cmd_response_window, "STATUS:2")) {

        // Not a hard fault; trigger retry path
        state_inputs.response_ok = false;
        state_inputs.fault_present = false;     // <-- important
        state_inputs.command_sent = false;
        return;
    }

    // Anything else treat as fault
    state_inputs.response_ok = false;
    state_inputs.fault_present = true;
    state_inputs.command_sent = false;
}

STATIC void wifi_state_send_cipmode_on_entry(uint16_t prevState)
{
    (void)prevState;
    
    // Clear state inputs to ensure clean transition
    state_inputs.command_sent = false;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;
    
}

STATIC void wifi_state_wait_cipmode_process(void)
{
    // Wait 500ms after entering state before sending command
    // This ensures previous command responses are fully received
    if (wifi_state_machine.timer < MS_TO_10HZ_TICKS(500)) {
        return;
    }
    
    // Send command once after delay
    if (!state_inputs.command_sent) {
        // Extra drain to ensure clean state
        uint8_t dump[256];
        while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}
        
        wifi_start_command("AT+CIPMODE=1\r\n");
        return;
    }
    
    // Wait for OK response
    wifi_check_response("OK", 2000);
    
    // Debug: print what we received if we got a response
    // if (state_inputs.response_received) {
    // }
}

STATIC void wifi_state_send_cipsend_on_entry(uint16_t prevState)
{
    (void)prevState;
    
    // Clear state inputs
    state_inputs.command_sent = false;
    state_inputs.response_received = false;
    state_inputs.response_ok = false;
    state_inputs.fault_present = false;
    
}

STATIC void wifi_state_wait_cipsend_process(void)
{
    // Wait 500ms after CIPMODE before sending CIPSEND
    if (wifi_state_machine.timer < MS_TO_10HZ_TICKS(500)) {
        return;
    }
    
    // Send command once after delay
    if (!state_inputs.command_sent) {
        // Extra drain to ensure clean state
        uint8_t dump[256];
        while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}
        
        wifi_start_command("AT+CIPSEND\r\n");
        return;
    }
    
    // Wait for '>' prompt indicating transparent mode is active
    if (state_inputs.response_received) {
        return;
    }

    // Check for timeout
    if ((xTaskGetTickCount() - cmd_start_time) > pdMS_TO_TICKS(2000)) {
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }

    // Drain RX looking for '>' prompt
    uint8_t tmp[256];
    size_t n;

    while ((n = xStreamBufferReceive(wifi_rx_stream(), tmp, sizeof(tmp), 0)) > 0) {
        // Make room in response window
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

        // Check for '>' prompt
        if (strchr(cmd_response_window, '>')) {
            state_inputs.response_received = true;
            state_inputs.response_ok = true;
            state_inputs.command_sent = false;
            return;
        }
    }

    // Check for errors
    if (strstr(cmd_response_window, "ERROR")) {
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }
}

STATIC void wifi_state_active_on_entry(uint16_t prevState)
{
    (void)prevState;
    
    // Drain any leftover RX data from initialization
    uint8_t dump[256];
    while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}
    
    // Clear TCP closed flag
    tcp_closed_detected = false;
    
}

STATIC void wifi_state_active_process(void)
{
    // Persistent rolling window for detecting ESP messages like "CLOSED"
    static char detect_window[128] = {0};
    static size_t detect_used = 0;
    static bool settling_complete = false;

    // Give ESP8266 500ms to settle into transparent mode and clear buffers
    if (!settling_complete) {
        if (wifi_state_machine.timer < MS_TO_10HZ_TICKS(500)) {
            // Still draining during settling period
            uint8_t dump[256];
            while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}
            return;
        }
        settling_complete = true;
    }

    // Transmit telemetry from queue
    wifi_transmit_telemetry_queue();

    // -------------------------------------------------
    // Process incoming data (OTA commands)
    // -------------------------------------------------

    uint8_t rx_buf[256];
    size_t rx_len = xStreamBufferReceive(wifi_rx_stream(), rx_buf, sizeof(rx_buf), 0);

    if (rx_len == 0) {
        return;
    }

    // -------------------------------------------------
    // Append to rolling window for token detection
    // -------------------------------------------------

    for (size_t i = 0; i < rx_len; i++) {

        if (detect_used < sizeof(detect_window) - 1) {
            detect_window[detect_used++] = rx_buf[i];
        } else {
            memmove(detect_window, detect_window + 1, sizeof(detect_window) - 2);
            detect_window[sizeof(detect_window) - 2] = rx_buf[i];
            detect_used = sizeof(detect_window) - 1;
        }

        detect_window[detect_used] = 0;
    }

    // -------------------------------------------------
    // Detect TCP CLOSED from ESP8266
    // -------------------------------------------------

    if (strstr(detect_window, "+++")) {

        tcp_closed_detected = true;

        detect_used = 0;
        detect_window[0] = 0;

        return;
    }

    // -------------------------------------------------
    // Feed bytes to OTA parser
    // -------------------------------------------------
    for (size_t i = 0; i < rx_len; i++) {
        ota_parser_process_byte(rx_buf[i]);
    }


}

STATIC void wifi_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;
}

/**
 * @brief Transmit telemetry events from queue to WiFi
 * @note Called from wifi_state_active_process() at 10Hz
 */
STATIC void wifi_transmit_telemetry_queue(void)
{
    telemetry_event_t telem;
    char tx_buf[256];
    
    // Drain telemetry queue and transmit (non-blocking)
    while (xQueueReceive(telemetry_queue, &telem, 0) == pdPASS)
    {
        int len = 0;
        
        switch (telem.type)
        {
            case TELEMETRY_EVENT_RANGING:
                len = snprintf(tx_buf, sizeof(tx_buf),
                    "RANGE,%.3f,%.1f,%u,%.2f,%.2f,%.2f,%d\r\n",
                    telem.data.ranging.distance_m,
                    telem.data.ranging.rssi_dbm,
                    telem.data.ranging.anchor_addr,
                    telem.data.ranging.anchor_position.x,
                    telem.data.ranging.anchor_position.y,
                    telem.data.ranging.anchor_position.z,
                    telem.data.ranging.anchor_position_valid ? 1 : 0);
                break;
                
            case TELEMETRY_EVENT_SENSOR_EVENT:
                if (telem.data.sensor_event.type == SENSOR_EVENT_IMU)
                {
                    len = snprintf(tx_buf, sizeof(tx_buf),
                        "IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\r\n",
                        telem.data.sensor_event.data.imu.accel_x,
                        telem.data.sensor_event.data.imu.accel_y,
                        telem.data.sensor_event.data.imu.accel_z,
                        telem.data.sensor_event.data.imu.gyro_x,
                        telem.data.sensor_event.data.imu.gyro_y,
                        telem.data.sensor_event.data.imu.gyro_z,
                        telem.data.sensor_event.data.imu.temp_c);
                }
                break;
                
            case TELEMETRY_EVENT_POSITION:
                len = snprintf(tx_buf, sizeof(tx_buf),
                    "POS,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%d\r\n",
                    telem.data.position.x,
                    telem.data.position.y,
                    telem.data.position.z,
                    telem.data.position.vx,
                    telem.data.position.vy,
                    telem.data.position.vz,
                    telem.data.position.confidence,
                    telem.data.position.valid ? 1 : 0,
                    telem.data.position.imu_enable);
                break;
                
            default:
                continue; // Skip unknown types
        }
        
        if (len > 0 && len < (int)sizeof(tx_buf))
        {
            //uart_driver_transmit(UART_WIFI, (uint8_t*)tx_buf, (uint16_t)len);
            telemetry_stats.events_transmitted++;
        }
    }
}

// Non-blocking command helpers
STATIC void wifi_start_command(const char *cmd)
{
    // Drain old RX data
    uint8_t dump[256];
    while (xStreamBufferReceive(wifi_rx_stream(), dump, sizeof(dump), 0) > 0) {}

    // Send command - use blocking for initialization commands
    // (wifi module runs in periodic callback, not dedicated task)
    uart_driver_transmit(UART_WIFI, (uint8_t*)cmd, (uint16_t)strlen(cmd));

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
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }

    uint8_t tmp[256];
    size_t n;

    // Drain ALL available bytes each call
    while ((n = xStreamBufferReceive(wifi_rx_stream(), tmp, sizeof(tmp), 0)) > 0) {
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
        state_inputs.response_received = true;
        state_inputs.response_ok = true;
        state_inputs.command_sent = false;
        return;
    }

    if (strstr(cmd_response_window, "ERROR")) {
        state_inputs.response_received = true;
        state_inputs.response_ok = false;
        state_inputs.fault_present = true;
        return;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

wifi_telemetry_status_e wifi_push_telemetry(const telemetry_event_t* event)
{
    if (event == NULL)
    {
        return WIFI_TELEMETRY_ERROR_NULL_PTR;
    }

    if (telemetry_queue == NULL)
    {
        return WIFI_TELEMETRY_ERROR_NOT_INITIALIZED;
    }

    if (event->type >= NUM_TELEMETRY_EVENT_TYPES)
    {
        return WIFI_TELEMETRY_ERROR_INVALID_TYPE;
    }

    // Only accept data when in ACTIVE state
    if (wifi_state_machine.curr_state != STATE_ACTIVE)
    {
        return WIFI_TELEMETRY_ERROR_NOT_CONNECTED;
    }

    telemetry_event_t queued_event = *event;

    taskENTER_CRITICAL();
    queued_event.sequence = telemetry_stats.sequence++;
    taskEXIT_CRITICAL();

    BaseType_t result = xQueueSend(telemetry_queue, &queued_event, 0);

    if (result == pdPASS)
    {
        telemetry_stats.events_pushed++;
        return WIFI_TELEMETRY_SUCCESS;
    }

    // Queue full - drop the event (non-blocking)
    telemetry_stats.events_dropped++;
    return WIFI_TELEMETRY_ERROR_QUEUE_FULL;
}

bool wifi_telemetry_is_ready(void)
{
    return (telemetry_queue != NULL && wifi_state_machine.curr_state == STATE_ACTIVE);
}


