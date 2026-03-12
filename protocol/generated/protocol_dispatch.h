/* Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py */
#ifndef PROTOCOL_DISPATCH_H
#define PROTOCOL_DISPATCH_H
#include <stddef.h>
#include <stdint.h>
#include "protocol_ids.h"
#include "uart_protocol.pb.h"

void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len);

void protocol_rx_PingRequest(const PingRequest *msg);
void protocol_rx_PingResponse(const PingResponse *msg);
void protocol_rx_SetAddressRequest(const SetAddressRequest *msg);
void protocol_rx_SetAddressResponse(const SetAddressResponse *msg);
void protocol_rx_GetConfigRequest(const GetConfigRequest *msg);
void protocol_rx_GetConfigResponse(const GetConfigResponse *msg);
void protocol_rx_LogLine(const LogLine *msg);

#endif /* PROTOCOL_DISPATCH_H */
