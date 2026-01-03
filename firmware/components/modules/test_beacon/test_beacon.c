/*---------------------------------------------------------------------------
 * @file    test_beacon.c
 * @brief   Test beacon module implementation
 *
 * @note    This module demonstrates how to use the UWB API. It listens for
 *          incoming UWB messages and responds with an auto-incrementing counter.
 *          This is demo code and can be excluded from production builds.
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "test_beacon.h"
#include "mac_802154.h"
#include "module.h"
#include "uart_cmd_router.h"
#include "uart_manager.h"
#include "uwb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/
typedef enum
{
    MODE_RESPONDER, // Auto-respond to incoming messages (default)
    MODE_MASTER     // Only listen and send commands, don't auto-respond
} beacon_mode_t;

typedef struct
{
    beacon_mode_t mode;     // Operating mode (responder or master)
    uint16_t counter;       // Current counter value (increments on each response)
    uint32_t rx_count;      // Number of messages received
    uint32_t tx_count;      // Number of responses sent
    uint16_t last_src_addr; // Last sender's address
} beacon_state_t;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void test_beacon_init(void);
STATIC void test_beacon_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC bool test_beacon_cmd_handler(const cmd_parsed_t* parsed);

extern const module_S test_beacon_module;

const module_S test_beacon_module = {
    .module_name = "beacon",
    .module_init = test_beacon_init,
    .module_process_1Hz = NULL,
    .module_cmd_handler = test_beacon_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC beacon_state_t beacon = {
    .mode = MODE_RESPONDER, // Responder mode by default
    .counter = 0,
    .rx_count = 0,
    .tx_count = 0,
    .last_src_addr = 0,
};

/*---------------------------------------------------------------------------
 * Module Implementation
 *---------------------------------------------------------------------------*/
STATIC void test_beacon_init(void)
{
    // Register callback to receive UWB messages
    uwb_register_rx_callback(test_beacon_rx_callback);

    // Automatically start UWB radio
    uwb_start();
}

/**
 * @brief RX callback - called when a UWB message is received
 *
 * Responds to incoming messages with an auto-incrementing counter value.
 * The counter increments on each response sent.
 */
STATIC void test_beacon_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    // Track received message
    beacon.rx_count++;
    beacon.last_src_addr = src_addr;

    // Print received message with content
    uart_manager_print("RX from 0x%04X (%u bytes): ", src_addr, length);

    // Print the actual data as string (if printable) or hex
    bool is_printable = true;
    for (uint16_t i = 0; i < length; i++)
    {
        if (data[i] < 32 || data[i] > 126)
        {
            is_printable = false;
            break;
        }
    }

    if (is_printable && length > 0)
    {
        // Print as string
        uart_manager_print("\"");
        for (uint16_t i = 0; i < length; i++)
        {
            uart_manager_print("%c", data[i]);
        }
        uart_manager_print("\"\r\n");
    }
    else
    {
        // Print as hex
        for (uint16_t i = 0; i < length; i++)
        {
            uart_manager_print("%02X ", data[i]);
        }
        uart_manager_print("\r\n");
    }

    // Only respond if in responder mode and UWB is ready
    if (beacon.mode != MODE_RESPONDER || !uwb_is_ready())
    {
        return;
    }

    // Format counter value as string payload for response
    char response[6]; // Max 5 digits for uint16_t (65535) + null terminator
    snprintf(response, sizeof(response), "%u", beacon.counter);
    uint16_t response_len = strlen(response);

    // Send response back to the sender
    bool success = uwb_send_message((uint8_t*)response, response_len, src_addr);

    if (success)
    {
        beacon.tx_count++;
        uart_manager_print("TX response to 0x%04X: counter=%u\r\n", src_addr, beacon.counter);
        beacon.counter++; // Increment counter after each successful response
    }
    else
    {
        uart_manager_print("TX response failed\r\n");
    }
}

/*---------------------------------------------------------------------------
 * Public API Implementation
 *---------------------------------------------------------------------------*/
void test_beacon_enable(bool enable)
{
    beacon.mode = enable ? MODE_RESPONDER : MODE_MASTER;
}

bool test_beacon_is_enabled(void)
{
    return (beacon.mode == MODE_RESPONDER);
}

void test_beacon_set_value(uint16_t value)
{
    beacon.counter = value;
}

void test_beacon_set_auto_increment(bool enable)
{
    // No longer used - counter always auto-increments on response
    (void)enable;
}

void test_beacon_get_stats(uint16_t* counter, uint32_t* attempts, bool* auto_inc)
{
    if (counter != NULL)
    {
        *counter = beacon.counter;
    }
    if (attempts != NULL)
    {
        *attempts = beacon.tx_count;
    }
    if (auto_inc != NULL)
    {
        *auto_inc = true; // Always auto-increments
    }
}

/*---------------------------------------------------------------------------
 * Command Handler
 *---------------------------------------------------------------------------*/
STATIC bool test_beacon_cmd_handler(const cmd_parsed_t* parsed)
{
    switch (parsed->action)
    {
        case CMD_ACTION_GET:
            if (strcmp(parsed->target, "status") == 0)
            {
                const char* mode_str = (beacon.mode == MODE_RESPONDER) ? "responder" : "master";
                uart_manager_print("Beacon mode: %s\r\n", mode_str);
                return true;
            }
            else if (strcmp(parsed->target, "stats") == 0)
            {
                const char* mode_str = (beacon.mode == MODE_RESPONDER) ? "responder" : "master";
                uart_manager_print("Beacon stats:\r\n");
                uart_manager_print("  Mode: %s\r\n", mode_str);
                uart_manager_print("  Counter: %u\r\n", beacon.counter);
                uart_manager_print("  Messages received: %u\r\n", (unsigned int)beacon.rx_count);
                uart_manager_print("  Responses sent: %u\r\n", (unsigned int)beacon.tx_count);
                uart_manager_print("  Last sender: 0x%04X\r\n", beacon.last_src_addr);
                return true;
            }
            break;

        case CMD_ACTION_SET:
            if (strcmp(parsed->target, "mode") == 0)
            {
                if (strcmp(parsed->args, "responder") == 0)
                {
                    beacon.mode = MODE_RESPONDER;
                    uart_manager_print("Beacon mode: responder (auto-respond enabled)\r\n");
                    return true;
                }
                else if (strcmp(parsed->args, "master") == 0)
                {
                    beacon.mode = MODE_MASTER;
                    uart_manager_print("Beacon mode: master (auto-respond disabled)\r\n");
                    return true;
                }
                else
                {
                    uart_manager_print("Invalid mode. Use 'responder' or 'master'\r\n");
                    return true;
                }
            }
            else if (strcmp(parsed->target, "counter") == 0)
            {
                int value = atoi(parsed->args);
                if (value < 0 || value > 65535)
                {
                    uart_manager_print("Invalid value. Range: 0-65535\r\n");
                    return true;
                }
                beacon.counter = (uint16_t)value;
                uart_manager_print("Beacon counter reset to %u\r\n", value);
                return true;
            }
            break;

        case CMD_ACTION_REQ:
            if (strcmp(parsed->target, "ping") == 0)
            {
                // Send a ping message: "beacon.req ping [addr]"
                if (!uwb_is_ready())
                {
                    uart_manager_print("UWB not ready\r\n");
                    return true;
                }

                // Parse optional destination address (default: broadcast)
                uint16_t dest_addr = MAC_BROADCAST_ADDR;
                if (strlen(parsed->args) > 0)
                {
                    // Parse hex address (e.g., "0x0002" or just "2")
                    int addr = (int)strtol(parsed->args, NULL, 0);
                    if (addr >= 0 && addr <= 0xFFFF)
                    {
                        dest_addr = (uint16_t)addr;
                    }
                    else
                    {
                        uart_manager_print("Invalid address. Use 0x0000-0xFFFF\r\n");
                        return true;
                    }
                }

                // Send ping message
                const char* ping_msg = "ping";
                uart_manager_print("Sending ping to 0x%04X (len=%u)...\r\n", dest_addr,
                                   (unsigned int)strlen(ping_msg));
                bool success = uwb_send_message((uint8_t*)ping_msg, strlen(ping_msg), dest_addr);

                if (success)
                {
                    uart_manager_print("Ping sent successfully\r\n");
                }
                else
                {
                    uart_manager_print("Ping failed - TX error\r\n");
                }
                return true;
            }
            break;

        case CMD_ACTION_UNKNOWN:
        default:
            break;
    }

    return false;
}
