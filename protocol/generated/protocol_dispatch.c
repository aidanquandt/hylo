/* Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py */
#include "protocol_dispatch.h"
#include "uart_protocol.pb.h"
#include "protocol_ids.h"
#include <pb_decode.h>
#include <stddef.h>

__attribute__((weak)) void protocol_rx_PingRequest(const PingRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_PingResponse(const PingResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SetAddressRequest(const SetAddressRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SetAddressResponse(const SetAddressResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_GetConfigRequest(const GetConfigRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_GetConfigResponse(const GetConfigResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_LogLine(const LogLine *msg) { (void)msg; }

void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len)
{
  pb_istream_t stream = pb_istream_from_buffer(payload, len);
  switch (msg_id) {
    case MSG_ID_PingRequest: {
      PingRequest decoded = PingRequest_init_zero;
      if (pb_decode(&stream, PingRequest_fields, &decoded))
        protocol_rx_PingRequest(&decoded);
      break;
    }
    case MSG_ID_PingResponse: {
      PingResponse decoded = PingResponse_init_zero;
      if (pb_decode(&stream, PingResponse_fields, &decoded))
        protocol_rx_PingResponse(&decoded);
      break;
    }
    case MSG_ID_SetAddressRequest: {
      SetAddressRequest decoded = SetAddressRequest_init_zero;
      if (pb_decode(&stream, SetAddressRequest_fields, &decoded))
        protocol_rx_SetAddressRequest(&decoded);
      break;
    }
    case MSG_ID_SetAddressResponse: {
      SetAddressResponse decoded = SetAddressResponse_init_zero;
      if (pb_decode(&stream, SetAddressResponse_fields, &decoded))
        protocol_rx_SetAddressResponse(&decoded);
      break;
    }
    case MSG_ID_GetConfigRequest: {
      GetConfigRequest decoded = GetConfigRequest_init_zero;
      if (pb_decode(&stream, GetConfigRequest_fields, &decoded))
        protocol_rx_GetConfigRequest(&decoded);
      break;
    }
    case MSG_ID_GetConfigResponse: {
      GetConfigResponse decoded = GetConfigResponse_init_zero;
      if (pb_decode(&stream, GetConfigResponse_fields, &decoded))
        protocol_rx_GetConfigResponse(&decoded);
      break;
    }
    case MSG_ID_LogLine: {
      LogLine decoded = LogLine_init_zero;
      if (pb_decode(&stream, LogLine_fields, &decoded))
        protocol_rx_LogLine(&decoded);
      break;
    }
    default:
      break;
  }
}
