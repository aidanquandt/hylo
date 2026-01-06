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
#include "uart_manager.h"
#include "uwb.h"
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    beacon_mode_e mode;
    uint16_t counter;
    uint32_t rx_count;
    uint32_t tx_count;
    uint16_t last_src_addr;
} beacon_state_t;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void test_beacon_init(void);
STATIC void test_beacon_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr);

extern const module_S test_beacon_module;

const module_S test_beacon_module = {
    .module_name = "beacon",
    .module_init = test_beacon_init,
    .module_process_1Hz = NULL,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC beacon_state_t beacon = {
    .mode = BEACON_MODE_RESPONDER,
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

STATIC void test_beacon_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    beacon.rx_count++;
    beacon.last_src_addr = src_addr;

    // Responder mode: send auto-incrementing counter back
    if (beacon.mode == BEACON_MODE_RESPONDER && uwb_is_ready())
    {
        char response[6];
        snprintf(response, sizeof(response), "%u", beacon.counter);
        uint16_t response_len = strlen(response);

        if (uwb_send_message((uint8_t*)response, response_len, src_addr))
        {
            beacon.tx_count++;
            beacon.counter++;
        }
    }
}

/*---------------------------------------------------------------------------
 * Public API Implementation
 *---------------------------------------------------------------------------*/

void test_beacon_get_status(beacon_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->mode = beacon.mode;
    status->counter = beacon.counter;
    status->rx_count = beacon.rx_count;
    status->tx_count = beacon.tx_count;
    status->last_src_addr = beacon.last_src_addr;
}

void test_beacon_set_mode(beacon_mode_e mode)
{
    beacon.mode = mode;
}

void test_beacon_set_counter(uint16_t value)
{
    beacon.counter = value;
}

bool test_beacon_send_ping(uint16_t dest_addr)
{
    if (!uwb_is_ready())
    {
        return false;
    }

    const char* ping_msg = "ping";
    return uwb_send_message((uint8_t*)ping_msg, strlen(ping_msg), dest_addr);
}
