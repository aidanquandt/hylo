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
 * Typedefs
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
STATIC void beacon_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                    uint64_t rx_timestamp);

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
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void test_beacon_init(void)
{
    // Register DATA protocol handler with UWB module
    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_DATA, beacon_protocol_handler))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "beacon",
                          "Failed to register DATA protocol handler");
    }

    uwb_start();
}

STATIC void beacon_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                    uint64_t rx_timestamp)
{
    (void)rx_timestamp; // Not used for beacon
    beacon.rx_count++;
    beacon.last_src_addr = src_addr;

    if (length < sizeof(protocol_header_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "beacon", "Invalid RX message (too short)");
        return;
    }

    const protocol_header_t* hdr = (const protocol_header_t*)data;

    if (hdr->msg_type != DATA_MSG_TYPE_BEACON)
    {
        return;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
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
    if (!uwb_is_ready())
    {
        return false;
    }

    protocol_data_beacon_msg_t ping;
    ping.header.protocol_type = PROTOCOL_TYPE_DATA;
    ping.header.msg_type = DATA_MSG_TYPE_BEACON;
    ping.header.sequence = beacon.counter++;
    ping.counter = beacon.counter;

    uwb_dev_t* dev = uwb_get_device();
    if (dev == NULL)
    {
        return false;
    }

    uint32_t current_time_dtuh = (uint32_t)uwb_port_read_device_time();
    uint32_t delay_dtuh = (uint32_t)UWB_MS_TO_DTUH(20);
    uint32_t tx_time_dtuh = current_time_dtuh + delay_dtuh;

    bool result = uwb_send_message_delayed((uint8_t*)&ping, sizeof(ping), dest_addr, tx_time_dtuh);

    return result;
}
