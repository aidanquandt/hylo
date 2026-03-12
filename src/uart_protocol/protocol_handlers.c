/*
 * UART protocol RX handlers: Ping, GetConfig, SetAddress.
 * Strong implementations override weak stubs in protocol_dispatch.c.
 */
#include "protocol_dispatch.h"
#include "protocol_tx.h"
#include "uart_protocol.pb.h"
#include "uwb.h"
#include <stdbool.h>
#include <stdint.h>

static uint32_t s_ping_seq;

void protocol_rx_PingRequest(const PingRequest *msg)
{
    (void)msg;
    PingResponse r = PingResponse_init_zero;
    r.seq = s_ping_seq++;
    protocol_tx_PingResponse(&r);
}

void protocol_rx_GetConfigRequest(const GetConfigRequest *msg)
{
    (void)msg;
    GetConfigResponse r = GetConfigResponse_init_zero;
    r.pan_id = uwb_get_pan_id();
    r.short_addr = uwb_get_address();
    protocol_tx_GetConfigResponse(&r);
}

void protocol_rx_SetAddressRequest(const SetAddressRequest *msg)
{
    SetAddressResponse r = SetAddressResponse_init_zero;
    uwb_set_address((uint16_t)msg->address, (uint16_t)msg->pan_id);
    r.success = true;
    protocol_tx_SetAddressResponse(&r);
}
