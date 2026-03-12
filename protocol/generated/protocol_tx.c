/* Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py */
#include "protocol_tx.h"
#include "uart_protocol.pb.h"
#include "uart_framing.h"
#include "protocol_ids.h"
#include <pb_encode.h>
#include <stddef.h>

/* Single encode buffer; size must fit largest message (see UART_PROTOCOL_PB_H_MAX_SIZE). */
#define PROTOCOL_TX_BUF_SIZE 80
static uint8_t s_tx_buf[PROTOCOL_TX_BUF_SIZE];

void protocol_tx_PingRequest(const PingRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, PingRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_PingRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_PingResponse(const PingResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, PingResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_PingResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SetAddressRequest(const SetAddressRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SetAddressRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SetAddressRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SetAddressResponse(const SetAddressResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SetAddressResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SetAddressResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_GetConfigRequest(const GetConfigRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, GetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_GetConfigRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_GetConfigResponse(const GetConfigResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, GetConfigResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_GetConfigResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_LogLine(const LogLine *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, LogLine_fields, msg))
    return;
  protocol_send_frame(MSG_ID_LogLine, s_tx_buf, stream.bytes_written);
}

