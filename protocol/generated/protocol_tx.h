/* Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py */
#ifndef PROTOCOL_TX_H
#define PROTOCOL_TX_H
#include "protocol_ids.h"
#include "uart_protocol.pb.h"

void protocol_tx_PingRequest(const PingRequest *msg);
void protocol_tx_PingResponse(const PingResponse *msg);
void protocol_tx_SetAddressRequest(const SetAddressRequest *msg);
void protocol_tx_SetAddressResponse(const SetAddressResponse *msg);
void protocol_tx_GetConfigRequest(const GetConfigRequest *msg);
void protocol_tx_GetConfigResponse(const GetConfigResponse *msg);
void protocol_tx_LogLine(const LogLine *msg);

#endif /* PROTOCOL_TX_H */
