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
#include "error_handler.h"
#include "mac_802154.h"
#include "module.h"
#include "platform_timer.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_port.h"
#include "uwb_protocol_messages.h"
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
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void beacon_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void test_beacon_init(void);

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
    // Register DATA protocol handler with UWB module
    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_DATA, beacon_protocol_handler))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "beacon",
                          "Failed to register DATA protocol handler");
    }

    // Automatically start UWB radio
    uwb_start();
}

STATIC void beacon_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    beacon.rx_count++;
    beacon.last_src_addr = src_addr;

    // Capture device time and RX message timestamp
    // NOTE: read_device_time() returns DTUH (high 32-bits only)
    //       get_last_rx_timestamp() returns DTU (full 40-bit value)
    uint32_t device_time_dtuh = (uint32_t)uwb_port_read_device_time();
    uwb_dev_t* dev = uwb_get_device();
    uint64_t rx_msg_timestamp_dtu = (dev != NULL) ? uwb_port_get_last_rx_timestamp(dev) : 0;
    uint32_t rx_msg_timestamp_dtuh = (uint32_t)(rx_msg_timestamp_dtu >> 8);

    // Validate minimum message size
    if (length < sizeof(protocol_header_t))
    {
        uart_manager_print("Beacon RX: Invalid message (too short)\r\n");
        return;
    }

    const protocol_header_t* hdr = (const protocol_header_t*)data;

    // Only handle beacon messages
    if (hdr->msg_type != DATA_MSG_TYPE_BEACON)
    {
        return;
    }

    // Parse beacon message
    if (length >= sizeof(protocol_data_beacon_msg_t))
    {
        const protocol_data_beacon_msg_t* msg = (const protocol_data_beacon_msg_t*)data;
        uart_manager_print("Beacon RX: From 0x%04X, Seq=%u, Counter=%u, DevTime=%lu (dtuh), "
                           "RxTime=%lu DTU (dtuh=%lu)\r\n",
                           src_addr, hdr->sequence, msg->counter, device_time_dtuh,
                           (uint32_t)rx_msg_timestamp_dtu, rx_msg_timestamp_dtuh);
    }
    else
    {
        uart_manager_print(
            "Beacon RX: From 0x%04X, Seq=%u, DevTime=%lu (dtuh), RxTime=%lu DTU (dtuh=%lu)\r\n",
            src_addr, hdr->sequence, device_time_dtuh, (uint32_t)rx_msg_timestamp_dtu,
            rx_msg_timestamp_dtuh);
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

    protocol_data_beacon_msg_t ping;
    ping.header.protocol_type = PROTOCOL_TYPE_DATA;
    ping.header.msg_type = DATA_MSG_TYPE_BEACON;
    ping.header.sequence = beacon.counter++;
    ping.counter = beacon.counter;

    return uwb_send_message((uint8_t*)&ping, sizeof(ping), dest_addr);
}

bool test_beacon_send_ping_delayed(uint16_t dest_addr)
{
    uint32_t time1 = platform_get_timestamp();
    if (!uwb_is_ready())
    {
        return false;
    }

    protocol_data_beacon_msg_t ping;
    ping.header.protocol_type = PROTOCOL_TYPE_DATA;
    ping.header.msg_type = DATA_MSG_TYPE_BEACON;
    ping.header.sequence = beacon.counter++;
    ping.counter = beacon.counter;

    // Read current system time and calculate absolute TX time
    uwb_dev_t* dev = uwb_get_device();
    if (dev == NULL)
    {
        return false;
    }

    // NOTE: read_device_time() returns DTUH (high 32-bits only)
    // For delayed TX, we need to pass DTUH to uwb_send_message_delayed()
    uint32_t current_time_dtuh = (uint32_t)uwb_port_read_device_time();
    uint32_t delay_dtuh = (uint32_t)UWB_MS_TO_DTUH(20);
    uint32_t tx_time_dtuh = current_time_dtuh + delay_dtuh; // Add DTUH + DTUH

    uart_manager_print("Beacon delayed TX: curr_dtuh=%lu, delay_dtuh=%lu, tx_dtuh=%lu\r\n",
                       current_time_dtuh, delay_dtuh, tx_time_dtuh);

    uint32_t time2 = platform_get_timestamp();
    // Pass DTUH (tx_time_dtuh) to delayed send function
    bool result = uwb_send_message_delayed((uint8_t*)&ping, sizeof(ping), dest_addr, tx_time_dtuh);

    uint32_t time3 = platform_get_timestamp();

    uint32_t elapsed1 = platform_get_elapsed_us(time1, time2);
    uint32_t elapsed2 = platform_get_elapsed_us(time2, time3);

    uart_manager_print("Beacon delayed TX: prep=%lu us, send=%lu us\r\n", elapsed1, elapsed2);

    return result;
}
