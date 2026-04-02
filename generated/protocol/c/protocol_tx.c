/* Auto-generated from protocol/protocol.proto barrel by tools/protocol_codegen/codegen_protocol.py */
#include "protocol_tx.h"
#include "protocol.pb.h"
#include "uart_framing.h"
#include "protocol_ids.h"
#include <pb_encode.h>
#include <stddef.h>

/* Stack-local encode buffer per call: avoids shared-state races across tasks. */
#define PROTOCOL_TX_BUF_SIZE 110

void protocol_tx_AckResponse(const AckResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, AckResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AckResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_TransportSetRequest(const TransportSetRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TransportSetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TransportSetRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TransportGetRequest(const TransportGetRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TransportGetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TransportGetRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TransportGetResponse(const TransportGetResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TransportGetResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TransportGetResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetUuidRequest(const SystemGetUuidRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SystemGetUuidRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetUuidRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetUuidResponse(const SystemGetUuidResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SystemGetUuidResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetUuidResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetInfoRequest(const SystemGetInfoRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SystemGetInfoRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetInfoRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetInfoResponse(const SystemGetInfoResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SystemGetInfoResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetInfoResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_PingRequest(const PingRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, PingRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_PingRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_PingResponse(const PingResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, PingResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_PingResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SetAddressRequest(const SetAddressRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SetAddressRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SetAddressRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SetAddressResponse(const SetAddressResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SetAddressResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SetAddressResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_GetConfigRequest(const GetConfigRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, GetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_GetConfigRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_GetConfigResponse(const GetConfigResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, GetConfigResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_GetConfigResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetStatusRequest(const ImuGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetStatusResponse(const ImuGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetDataRequest(const ImuGetDataRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuGetDataRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetDataRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetDataResponse(const ImuGetDataResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuGetDataResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetDataResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamStartRequest(const ImuStreamStartRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuStreamStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamStartRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamStopRequest(const ImuStreamStopRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuStreamStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamStopRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuCalibrateRequest(const ImuCalibrateRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuCalibrateRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuCalibrateRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuCalibrateResponse(const ImuCalibrateResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuCalibrateResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuCalibrateResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuCalibrateCompleteEvent(const ImuCalibrateCompleteEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuCalibrateCompleteEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuCalibrateCompleteEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamPayload(const ImuStreamPayload *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuStreamPayload_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamPayload, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeSetTypeRequest(const UwbNodeSetTypeRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeSetTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeSetTypeRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetTypeRequest(const UwbNodeGetTypeRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetTypeRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetTypeResponse(const UwbNodeGetTypeResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetTypeResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetTypeResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeSetPositionRequest(const UwbNodeSetPositionRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeSetPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeSetPositionRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetPositionRequest(const UwbNodeGetPositionRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetPositionRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetPositionResponse(const UwbNodeGetPositionResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetPositionResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetPositionResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetStatusRequest(const UwbNodeGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetStatusResponse(const UwbNodeGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatusRequest(const UwbGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatusResponse(const UwbGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatsRequest(const UwbGetStatsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatsResponse(const UwbGetStatsResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatsResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbStartRequest(const UwbStartRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbStartRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbStopRequest(const UwbStopRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbStopRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbResetStatsRequest(const UwbResetStatsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbResetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbResetStatsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_ErrorClearRequest(const ErrorClearRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ErrorClearRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ErrorClearRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetTasksRequest(const DataloggerGetTasksRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetTasksRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetTasksRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetTasksResponse(const DataloggerGetTasksResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetTasksResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetTasksResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetStatsRequest(const DataloggerGetStatsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetStatsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetStatsResponse(const DataloggerGetStatsResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetStatsResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetStatusRequest(const TwrGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetStatusResponse(const TwrGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetResultRequest(const TwrGetResultRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrGetResultRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetResultRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetResultResponse(const TwrGetResultResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrGetResultResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetResultResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrRangeRequest(const TwrRangeRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrRangeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrRangeRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrStartRequest(const TwrMgrStartRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrStartRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrStopRequest(const TwrMgrStopRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrStopRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetStatusRequest(const TwrMgrGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetStatusResponse(const TwrMgrGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrAddTargetRequest(const TwrMgrAddTargetRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrAddTargetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrAddTargetRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRemoveTargetRequest(const TwrMgrRemoveTargetRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrRemoveTargetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRemoveTargetRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrSetTargetsRequest(const TwrMgrSetTargetsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrSetTargetsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrSetTargetsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrClearTargetsRequest(const TwrMgrClearTargetsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrClearTargetsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrClearTargetsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrSetRangingRateRequest(const TwrMgrSetRangingRateRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrSetRangingRateRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrSetRangingRateRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetRangingRateRequest(const TwrMgrGetRangingRateRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrGetRangingRateRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetRangingRateRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetRangingRateResponse(const TwrMgrGetRangingRateResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrGetRangingRateResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetRangingRateResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchGetRequest(const StopwatchGetRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, StopwatchGetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchGetRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchGetResponse(const StopwatchGetResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, StopwatchGetResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchGetResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchStartRequest(const StopwatchStartRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, StopwatchStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchStartRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchStopRequest(const StopwatchStopRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, StopwatchStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchStopRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendAddressRequest(const OtaConfigSendAddressRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigSendAddressRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendAddressRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendPositionRequest(const OtaConfigSendPositionRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigSendPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendPositionRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendTypeRequest(const OtaConfigSendTypeRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigSendTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendTypeRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendGpioRequest(const OtaConfigSendGpioRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigSendGpioRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendGpioRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSetTokenRequest(const OtaConfigSetTokenRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigSetTokenRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSetTokenRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetTokenRequest(const OtaConfigGetTokenRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigGetTokenRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetTokenRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetTokenResponse(const OtaConfigGetTokenResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigGetTokenResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetTokenResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetStatsRequest(const OtaConfigGetStatsRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetStatsRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetStatsResponse(const OtaConfigGetStatsResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetStatsResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetDebugRequest(const SensorFusionGetDebugRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetDebugRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetDebugRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetDebugRequest(const SensorFusionSetDebugRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionSetDebugRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetDebugRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetStatusRequest(const SensorFusionGetStatusRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetStatusRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetStatusResponse(const SensorFusionGetStatusResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetStatusResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetActiveRequest(const SensorFusionSetActiveRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionSetActiveRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetActiveRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetImuEnabledRequest(const SensorFusionGetImuEnabledRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetImuEnabledRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetImuEnabledRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetImuEnabledResponse(const SensorFusionGetImuEnabledResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetImuEnabledResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetImuEnabledResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetImuEnabledRequest(const SensorFusionSetImuEnabledRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionSetImuEnabledRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetImuEnabledRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetNoiseRequest(const SensorFusionSetNoiseRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionSetNoiseRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetNoiseRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetNoiseRequest(const SensorFusionGetNoiseRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetNoiseRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetNoiseRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetNoiseResponse(const SensorFusionGetNoiseResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetNoiseResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetNoiseResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetConfigRequest(const SensorFusionGetConfigRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionGetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetConfigRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetConfigRequest(const SensorFusionSetConfigRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SensorFusionSetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetConfigRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_BeaconPingRequest(const BeaconPingRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, BeaconPingRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_BeaconPingRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_BeaconPingResponse(const BeaconPingResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, BeaconPingResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_BeaconPingResponse, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuInitReturnedNullEvent(const ImuInitReturnedNullEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuInitReturnedNullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuInitReturnedNullEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuProbeInitFailedEvent(const ImuProbeInitFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuProbeInitFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuProbeInitFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuInvalidChipIdEvent(const ImuInvalidChipIdEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuInvalidChipIdEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuInvalidChipIdEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuAccelConfigFailedEvent(const ImuAccelConfigFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuAccelConfigFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuAccelConfigFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGyroConfigFailedEvent(const ImuGyroConfigFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuGyroConfigFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGyroConfigFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuFaultEvent(const ImuFaultEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuFaultEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ImuFailedToPushEventToSensorFusionEvent(const ImuFailedToPushEventToSensorFusionEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ImuFailedToPushEventToSensorFusionEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuFailedToPushEventToSensorFusionEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigUnknownMessageTypeEvent(const OtaConfigUnknownMessageTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigUnknownMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigUnknownMessageTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidAddressEvent(const OtaConfigInvalidAddressEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidAddressEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidAddressEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAddressChangedEvent(const OtaConfigAddressChangedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigAddressChangedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAddressChangedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigPositionSetEvent(const OtaConfigPositionSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigPositionSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigPositionSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidNodeTypeEvent(const OtaConfigInvalidNodeTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidNodeTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidNodeTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigNodeTypeSetEvent(const OtaConfigNodeTypeSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigNodeTypeSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigNodeTypeSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidGpioPinEvent(const OtaConfigInvalidGpioPinEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidGpioPinEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidGpioPinEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidGpioStateEvent(const OtaConfigInvalidGpioStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidGpioStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidGpioStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGpioSetEvent(const OtaConfigGpioSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigGpioSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGpioSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigResponseEvent(const OtaConfigResponseEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigResponseEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigResponseEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAuthFailedEvent(const OtaConfigAuthFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigAuthFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAuthFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAckFromEvent(const OtaConfigAckFromEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigAckFromEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAckFromEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigNoAckFromEvent(const OtaConfigNoAckFromEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigNoAckFromEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigNoAckFromEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAuthTokenSetEvent(const OtaConfigAuthTokenSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, OtaConfigAuthTokenSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAuthTokenSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbFrameTooSmallEvent(const UwbFrameTooSmallEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbFrameTooSmallEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbFrameTooSmallEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbPayloadTooLargeEvent(const UwbPayloadTooLargeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbPayloadTooLargeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbPayloadTooLargeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbRetryAttemptEvent(const UwbRetryAttemptEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbRetryAttemptEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbRetryAttemptEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbFaultManualStopRequiredEvent(const UwbFaultManualStopRequiredEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbFaultManualStopRequiredEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbFaultManualStopRequiredEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbInvalidProtocolMessageEvent(const UwbInvalidProtocolMessageEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbInvalidProtocolMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbInvalidProtocolMessageEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbInvalidProtocolTypeEvent(const UwbInvalidProtocolTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbInvalidProtocolTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbInvalidProtocolTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbTxQueueFullEvent(const UwbTxQueueFullEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbTxQueueFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbTxQueueFullEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbProtocolHandlerTableFullEvent(const UwbProtocolHandlerTableFullEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbProtocolHandlerTableFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbProtocolHandlerTableFullEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeInvalidTypeEvent(const UwbNodeInvalidTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeInvalidTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeInvalidTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeTypeSetEvent(const UwbNodeTypeSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodeTypeSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeTypeSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodePositionSetEvent(const UwbNodePositionSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, UwbNodePositionSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodePositionSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrFailedToRegisterProtocolHandlerEvent(const TwrFailedToRegisterProtocolHandlerEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrFailedToRegisterProtocolHandlerEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrFailedToRegisterProtocolHandlerEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrResponderAutoStartFailedEvent(const TwrResponderAutoStartFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrResponderAutoStartFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrResponderAutoStartFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrModuleNotInitializedEvent(const TwrModuleNotInitializedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrModuleNotInitializedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrModuleNotInitializedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFailedToSendMessageEvent(const ResponderFailedToSendMessageEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderFailedToSendMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFailedToSendMessageEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderPollTooShortEvent(const ResponderPollTooShortEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderPollTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderPollTooShortEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFinalTooShortEvent(const ResponderFinalTooShortEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderFinalTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFinalTooShortEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFailedToSendFinalAckEvent(const ResponderFailedToSendFinalAckEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderFailedToSendFinalAckEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFailedToSendFinalAckEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderUnexpectedMessageTypeEvent(const ResponderUnexpectedMessageTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderUnexpectedMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderUnexpectedMessageTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderTimeoutWaitingForMessageEvent(const ResponderTimeoutWaitingForMessageEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderTimeoutWaitingForMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderTimeoutWaitingForMessageEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFaultEvent(const ResponderFaultEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFaultEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderCompletedRangingEvent(const ResponderCompletedRangingEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, ResponderCompletedRangingEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderCompletedRangingEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFailedToSendMessageEvent(const InitiatorFailedToSendMessageEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorFailedToSendMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFailedToSendMessageEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorResponseTooShortEvent(const InitiatorResponseTooShortEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorResponseTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorResponseTooShortEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFinalAckTooShortEvent(const InitiatorFinalAckTooShortEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorFinalAckTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFinalAckTooShortEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorUnexpectedMessageTypeEvent(const InitiatorUnexpectedMessageTypeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorUnexpectedMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorUnexpectedMessageTypeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFaultEvent(const InitiatorFaultEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFaultEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorDistanceCalculationFailedEvent(const InitiatorDistanceCalculationFailedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorDistanceCalculationFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorDistanceCalculationFailedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorUwbNotReadyEvent(const InitiatorUwbNotReadyEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, InitiatorUwbNotReadyEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorUwbNotReadyEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompletionTimeoutEvent(const TwrSmTxCompletionTimeoutEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompletionTimeoutEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompletionTimeoutEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompleteWrongStateEvent(const TwrSmTxCompleteWrongStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompleteWrongStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompleteWrongStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompleteMsgIdMismatchEvent(const TwrSmTxCompleteMsgIdMismatchEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompleteMsgIdMismatchEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompleteMsgIdMismatchEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmRxMessageWrongStateEvent(const TwrSmRxMessageWrongStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSmRxMessageWrongStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmRxMessageWrongStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTimeoutUnexpectedStateEvent(const TwrSmTimeoutUnexpectedStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSmTimeoutUnexpectedStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTimeoutUnexpectedStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerStrategyNotImplementedEvent(const TwrSchedulerStrategyNotImplementedEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerStrategyNotImplementedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerStrategyNotImplementedEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetListFullEvent(const TwrSchedulerTargetListFullEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetListFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetListFullEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetAlreadyExistsEvent(const TwrSchedulerTargetAlreadyExistsEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetAlreadyExistsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetAlreadyExistsEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerAddedTargetEvent(const TwrSchedulerAddedTargetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerAddedTargetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerAddedTargetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerRemovedTargetEvent(const TwrSchedulerRemovedTargetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerRemovedTargetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerRemovedTargetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerNullAddressesPointerEvent(const TwrSchedulerNullAddressesPointerEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerNullAddressesPointerEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerNullAddressesPointerEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetCountExceedsMaximumEvent(const TwrSchedulerTargetCountExceedsMaximumEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetCountExceedsMaximumEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetCountExceedsMaximumEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerSetTargetsEvent(const TwrSchedulerSetTargetsEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerSetTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerSetTargetsEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerClearedAllTargetsEvent(const TwrSchedulerClearedAllTargetsEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerClearedAllTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerClearedAllTargetsEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetEnabledDisabledEvent(const TwrSchedulerTargetEnabledDisabledEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetEnabledDisabledEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetEnabledDisabledEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetBackingOffEvent(const TwrSchedulerTargetBackingOffEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetBackingOffEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetBackingOffEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerStrategySetEvent(const TwrSchedulerStrategySetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrSchedulerStrategySetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerStrategySetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrFailedToPushRangingEventEvent(const TwrMgrFailedToPushRangingEventEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrFailedToPushRangingEventEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrFailedToPushRangingEventEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRangeEvent(const TwrMgrRangeEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrRangeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRangeEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringIdleStateEvent(const TwrMgrEnteringIdleStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringIdleStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringIdleStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringRangingStateEvent(const TwrMgrEnteringRangingStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringRangingStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringRangingStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringFaultedStateEvent(const TwrMgrEnteringFaultedStateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringFaultedStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringFaultedStateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrCannotStartNoTargetsEvent(const TwrMgrCannotStartNoTargetsEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrCannotStartNoTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrCannotStartNoTargetsEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrFailedToStartInitiatorEvent(const TwrMgrFailedToStartInitiatorEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrFailedToStartInitiatorEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrFailedToStartInitiatorEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrInvalidRangingRateEvent(const TwrMgrInvalidRangingRateEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrInvalidRangingRateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrInvalidRangingRateEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRangingRateSetEvent(const TwrMgrRangingRateSetEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, TwrMgrRangingRateSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRangingRateSetEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerTimingMissesEvent(const DataloggerTimingMissesEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerTimingMissesEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerTimingMissesEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerLowMemoryEvent(const DataloggerLowMemoryEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerLowMemoryEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerLowMemoryEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_AppDeviceNotInMappingTableEvent(const AppDeviceNotInMappingTableEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, AppDeviceNotInMappingTableEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppDeviceNotInMappingTableEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_AppUsingDefaultAddressEvent(const AppUsingDefaultAddressEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, AppUsingDefaultAddressEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppUsingDefaultAddressEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_AppFailedToInitDeviceIdEvent(const AppFailedToInitDeviceIdEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, AppFailedToInitDeviceIdEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppFailedToInitDeviceIdEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_WatchdogTaskFailureEvent(const WatchdogTaskFailureEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, WatchdogTaskFailureEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_WatchdogTaskFailureEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_SystemFatalEvent(const SystemFatalEvent *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, SystemFatalEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemFatalEvent, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetIdleCpuRequest(const DataloggerGetIdleCpuRequest *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetIdleCpuRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetIdleCpuRequest, tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetIdleCpuResponse(const DataloggerGetIdleCpuResponse *msg)
{
  uint8_t tx_buf[PROTOCOL_TX_BUF_SIZE];
  pb_ostream_t stream = pb_ostream_from_buffer(tx_buf, sizeof(tx_buf));
  if (!pb_encode(&stream, DataloggerGetIdleCpuResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetIdleCpuResponse, tx_buf, stream.bytes_written);
}

