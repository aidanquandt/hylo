/* Auto-generated from protocol.proto by tools/protocol_codegen/codegen_protocol.py */
#include "protocol_tx.h"
#include "protocol.pb.h"
#include "uart_framing.h"
#include "protocol_ids.h"
#include <pb_encode.h>
#include <stddef.h>

/* Single encode buffer; size must fit largest message (see PROTOCOL_PB_H_MAX_SIZE). */
#define PROTOCOL_TX_BUF_SIZE 110
static uint8_t s_tx_buf[PROTOCOL_TX_BUF_SIZE];

void protocol_tx_AckResponse(const AckResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, AckResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AckResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetUuidRequest(const SystemGetUuidRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SystemGetUuidRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetUuidRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetUuidResponse(const SystemGetUuidResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SystemGetUuidResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetUuidResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetInfoRequest(const SystemGetInfoRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SystemGetInfoRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetInfoRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SystemGetInfoResponse(const SystemGetInfoResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SystemGetInfoResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemGetInfoResponse, s_tx_buf, stream.bytes_written);
}

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

void protocol_tx_ImuGetStatusRequest(const ImuGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetStatusResponse(const ImuGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetDataRequest(const ImuGetDataRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuGetDataRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetDataRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGetDataResponse(const ImuGetDataResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuGetDataResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGetDataResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamStartRequest(const ImuStreamStartRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuStreamStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamStartRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamStopRequest(const ImuStreamStopRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuStreamStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamStopRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuStreamPayload(const ImuStreamPayload *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuStreamPayload_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuStreamPayload, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeSetTypeRequest(const UwbNodeSetTypeRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeSetTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeSetTypeRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetTypeRequest(const UwbNodeGetTypeRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetTypeRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetTypeResponse(const UwbNodeGetTypeResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetTypeResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetTypeResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeSetPositionRequest(const UwbNodeSetPositionRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeSetPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeSetPositionRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetPositionRequest(const UwbNodeGetPositionRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetPositionRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetPositionResponse(const UwbNodeGetPositionResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetPositionResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetPositionResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetStatusRequest(const UwbNodeGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeGetStatusResponse(const UwbNodeGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatusRequest(const UwbGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatusResponse(const UwbGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatsRequest(const UwbGetStatsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbGetStatsResponse(const UwbGetStatsResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbGetStatsResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbStartRequest(const UwbStartRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbStartRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbStopRequest(const UwbStopRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbStopRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbResetStatsRequest(const UwbResetStatsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbResetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbResetStatsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ErrorClearRequest(const ErrorClearRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ErrorClearRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ErrorClearRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetTasksRequest(const DataloggerGetTasksRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerGetTasksRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetTasksRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetTasksResponse(const DataloggerGetTasksResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerGetTasksResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetTasksResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetStatsRequest(const DataloggerGetStatsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetStatsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerGetStatsResponse(const DataloggerGetStatsResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerGetStatsResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetStatusRequest(const TwrGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetStatusResponse(const TwrGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetResultRequest(const TwrGetResultRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrGetResultRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetResultRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrGetResultResponse(const TwrGetResultResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrGetResultResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrGetResultResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrRangeRequest(const TwrRangeRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrRangeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrRangeRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrStartRequest(const TwrMgrStartRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrStartRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrStopRequest(const TwrMgrStopRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrStopRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetStatusRequest(const TwrMgrGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetStatusResponse(const TwrMgrGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrAddTargetRequest(const TwrMgrAddTargetRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrAddTargetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrAddTargetRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRemoveTargetRequest(const TwrMgrRemoveTargetRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrRemoveTargetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRemoveTargetRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrSetTargetsRequest(const TwrMgrSetTargetsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrSetTargetsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrSetTargetsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrClearTargetsRequest(const TwrMgrClearTargetsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrClearTargetsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrClearTargetsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrSetRangingRateRequest(const TwrMgrSetRangingRateRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrSetRangingRateRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrSetRangingRateRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetRangingRateRequest(const TwrMgrGetRangingRateRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrGetRangingRateRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetRangingRateRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrGetRangingRateResponse(const TwrMgrGetRangingRateResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrGetRangingRateResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrGetRangingRateResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchGetRequest(const StopwatchGetRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, StopwatchGetRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchGetRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchGetResponse(const StopwatchGetResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, StopwatchGetResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchGetResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchStartRequest(const StopwatchStartRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, StopwatchStartRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchStartRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_StopwatchStopRequest(const StopwatchStopRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, StopwatchStopRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_StopwatchStopRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendAddressRequest(const OtaConfigSendAddressRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigSendAddressRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendAddressRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendPositionRequest(const OtaConfigSendPositionRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigSendPositionRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendPositionRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendTypeRequest(const OtaConfigSendTypeRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigSendTypeRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendTypeRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSendGpioRequest(const OtaConfigSendGpioRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigSendGpioRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSendGpioRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigSetTokenRequest(const OtaConfigSetTokenRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigSetTokenRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigSetTokenRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetTokenRequest(const OtaConfigGetTokenRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigGetTokenRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetTokenRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetTokenResponse(const OtaConfigGetTokenResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigGetTokenResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetTokenResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetStatsRequest(const OtaConfigGetStatsRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigGetStatsRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetStatsRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGetStatsResponse(const OtaConfigGetStatsResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigGetStatsResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGetStatsResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetDebugRequest(const SensorFusionGetDebugRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetDebugRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetDebugRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetDebugRequest(const SensorFusionSetDebugRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionSetDebugRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetDebugRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetStatusRequest(const SensorFusionGetStatusRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetStatusRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetStatusRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetStatusResponse(const SensorFusionGetStatusResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetStatusResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetStatusResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetActiveRequest(const SensorFusionSetActiveRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionSetActiveRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetActiveRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetImuEnabledRequest(const SensorFusionGetImuEnabledRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetImuEnabledRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetImuEnabledRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetImuEnabledResponse(const SensorFusionGetImuEnabledResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetImuEnabledResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetImuEnabledResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetImuEnabledRequest(const SensorFusionSetImuEnabledRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionSetImuEnabledRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetImuEnabledRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetNoiseRequest(const SensorFusionSetNoiseRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionSetNoiseRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetNoiseRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetNoiseRequest(const SensorFusionGetNoiseRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetNoiseRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetNoiseRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetNoiseResponse(const SensorFusionGetNoiseResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetNoiseResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetNoiseResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionGetConfigRequest(const SensorFusionGetConfigRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionGetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionGetConfigRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SensorFusionSetConfigRequest(const SensorFusionSetConfigRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SensorFusionSetConfigRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SensorFusionSetConfigRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_BeaconPingRequest(const BeaconPingRequest *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, BeaconPingRequest_fields, msg))
    return;
  protocol_send_frame(MSG_ID_BeaconPingRequest, s_tx_buf, stream.bytes_written);
}

void protocol_tx_BeaconPingResponse(const BeaconPingResponse *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, BeaconPingResponse_fields, msg))
    return;
  protocol_send_frame(MSG_ID_BeaconPingResponse, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuInitReturnedNullEvent(const ImuInitReturnedNullEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuInitReturnedNullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuInitReturnedNullEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuProbeInitFailedEvent(const ImuProbeInitFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuProbeInitFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuProbeInitFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuInvalidChipIdEvent(const ImuInvalidChipIdEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuInvalidChipIdEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuInvalidChipIdEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuAccelConfigFailedEvent(const ImuAccelConfigFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuAccelConfigFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuAccelConfigFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuGyroConfigFailedEvent(const ImuGyroConfigFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuGyroConfigFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuGyroConfigFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuFaultEvent(const ImuFaultEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuFaultEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ImuFailedToPushEventToSensorFusionEvent(const ImuFailedToPushEventToSensorFusionEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ImuFailedToPushEventToSensorFusionEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ImuFailedToPushEventToSensorFusionEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigUnknownMessageTypeEvent(const OtaConfigUnknownMessageTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigUnknownMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigUnknownMessageTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidAddressEvent(const OtaConfigInvalidAddressEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidAddressEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidAddressEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAddressChangedEvent(const OtaConfigAddressChangedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigAddressChangedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAddressChangedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigPositionSetEvent(const OtaConfigPositionSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigPositionSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigPositionSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidNodeTypeEvent(const OtaConfigInvalidNodeTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidNodeTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidNodeTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigNodeTypeSetEvent(const OtaConfigNodeTypeSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigNodeTypeSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigNodeTypeSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidGpioPinEvent(const OtaConfigInvalidGpioPinEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidGpioPinEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidGpioPinEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigInvalidGpioStateEvent(const OtaConfigInvalidGpioStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigInvalidGpioStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigInvalidGpioStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigGpioSetEvent(const OtaConfigGpioSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigGpioSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigGpioSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigResponseEvent(const OtaConfigResponseEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigResponseEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigResponseEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAuthFailedEvent(const OtaConfigAuthFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigAuthFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAuthFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAckFromEvent(const OtaConfigAckFromEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigAckFromEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAckFromEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigNoAckFromEvent(const OtaConfigNoAckFromEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigNoAckFromEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigNoAckFromEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_OtaConfigAuthTokenSetEvent(const OtaConfigAuthTokenSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, OtaConfigAuthTokenSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_OtaConfigAuthTokenSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbFrameTooSmallEvent(const UwbFrameTooSmallEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbFrameTooSmallEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbFrameTooSmallEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbPayloadTooLargeEvent(const UwbPayloadTooLargeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbPayloadTooLargeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbPayloadTooLargeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbRetryAttemptEvent(const UwbRetryAttemptEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbRetryAttemptEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbRetryAttemptEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbFaultManualStopRequiredEvent(const UwbFaultManualStopRequiredEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbFaultManualStopRequiredEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbFaultManualStopRequiredEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbInvalidProtocolMessageEvent(const UwbInvalidProtocolMessageEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbInvalidProtocolMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbInvalidProtocolMessageEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbInvalidProtocolTypeEvent(const UwbInvalidProtocolTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbInvalidProtocolTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbInvalidProtocolTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbTxQueueFullEvent(const UwbTxQueueFullEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbTxQueueFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbTxQueueFullEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbProtocolHandlerTableFullEvent(const UwbProtocolHandlerTableFullEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbProtocolHandlerTableFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbProtocolHandlerTableFullEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeInvalidTypeEvent(const UwbNodeInvalidTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeInvalidTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeInvalidTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodeTypeSetEvent(const UwbNodeTypeSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodeTypeSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodeTypeSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_UwbNodePositionSetEvent(const UwbNodePositionSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, UwbNodePositionSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_UwbNodePositionSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrFailedToRegisterProtocolHandlerEvent(const TwrFailedToRegisterProtocolHandlerEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrFailedToRegisterProtocolHandlerEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrFailedToRegisterProtocolHandlerEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrResponderAutoStartFailedEvent(const TwrResponderAutoStartFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrResponderAutoStartFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrResponderAutoStartFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrModuleNotInitializedEvent(const TwrModuleNotInitializedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrModuleNotInitializedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrModuleNotInitializedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFailedToSendMessageEvent(const ResponderFailedToSendMessageEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderFailedToSendMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFailedToSendMessageEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderPollTooShortEvent(const ResponderPollTooShortEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderPollTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderPollTooShortEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFinalTooShortEvent(const ResponderFinalTooShortEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderFinalTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFinalTooShortEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFailedToSendFinalAckEvent(const ResponderFailedToSendFinalAckEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderFailedToSendFinalAckEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFailedToSendFinalAckEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderUnexpectedMessageTypeEvent(const ResponderUnexpectedMessageTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderUnexpectedMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderUnexpectedMessageTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderTimeoutWaitingForMessageEvent(const ResponderTimeoutWaitingForMessageEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderTimeoutWaitingForMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderTimeoutWaitingForMessageEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderFaultEvent(const ResponderFaultEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderFaultEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_ResponderCompletedRangingEvent(const ResponderCompletedRangingEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, ResponderCompletedRangingEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_ResponderCompletedRangingEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFailedToSendMessageEvent(const InitiatorFailedToSendMessageEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorFailedToSendMessageEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFailedToSendMessageEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorResponseTooShortEvent(const InitiatorResponseTooShortEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorResponseTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorResponseTooShortEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFinalAckTooShortEvent(const InitiatorFinalAckTooShortEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorFinalAckTooShortEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFinalAckTooShortEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorUnexpectedMessageTypeEvent(const InitiatorUnexpectedMessageTypeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorUnexpectedMessageTypeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorUnexpectedMessageTypeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorFaultEvent(const InitiatorFaultEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorFaultEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorFaultEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorDistanceCalculationFailedEvent(const InitiatorDistanceCalculationFailedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorDistanceCalculationFailedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorDistanceCalculationFailedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_InitiatorUwbNotReadyEvent(const InitiatorUwbNotReadyEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, InitiatorUwbNotReadyEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_InitiatorUwbNotReadyEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompletionTimeoutEvent(const TwrSmTxCompletionTimeoutEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompletionTimeoutEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompletionTimeoutEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompleteWrongStateEvent(const TwrSmTxCompleteWrongStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompleteWrongStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompleteWrongStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTxCompleteMsgIdMismatchEvent(const TwrSmTxCompleteMsgIdMismatchEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSmTxCompleteMsgIdMismatchEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTxCompleteMsgIdMismatchEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmRxMessageWrongStateEvent(const TwrSmRxMessageWrongStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSmRxMessageWrongStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmRxMessageWrongStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSmTimeoutUnexpectedStateEvent(const TwrSmTimeoutUnexpectedStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSmTimeoutUnexpectedStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSmTimeoutUnexpectedStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerStrategyNotImplementedEvent(const TwrSchedulerStrategyNotImplementedEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerStrategyNotImplementedEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerStrategyNotImplementedEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetListFullEvent(const TwrSchedulerTargetListFullEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetListFullEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetListFullEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetAlreadyExistsEvent(const TwrSchedulerTargetAlreadyExistsEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetAlreadyExistsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetAlreadyExistsEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerAddedTargetEvent(const TwrSchedulerAddedTargetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerAddedTargetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerAddedTargetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerRemovedTargetEvent(const TwrSchedulerRemovedTargetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerRemovedTargetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerRemovedTargetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerNullAddressesPointerEvent(const TwrSchedulerNullAddressesPointerEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerNullAddressesPointerEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerNullAddressesPointerEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetCountExceedsMaximumEvent(const TwrSchedulerTargetCountExceedsMaximumEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetCountExceedsMaximumEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetCountExceedsMaximumEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerSetTargetsEvent(const TwrSchedulerSetTargetsEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerSetTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerSetTargetsEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerClearedAllTargetsEvent(const TwrSchedulerClearedAllTargetsEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerClearedAllTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerClearedAllTargetsEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetEnabledDisabledEvent(const TwrSchedulerTargetEnabledDisabledEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetEnabledDisabledEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetEnabledDisabledEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerTargetBackingOffEvent(const TwrSchedulerTargetBackingOffEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerTargetBackingOffEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerTargetBackingOffEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrSchedulerStrategySetEvent(const TwrSchedulerStrategySetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrSchedulerStrategySetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrSchedulerStrategySetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrFailedToPushRangingEventEvent(const TwrMgrFailedToPushRangingEventEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrFailedToPushRangingEventEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrFailedToPushRangingEventEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRangeEvent(const TwrMgrRangeEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrRangeEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRangeEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringIdleStateEvent(const TwrMgrEnteringIdleStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringIdleStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringIdleStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringRangingStateEvent(const TwrMgrEnteringRangingStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringRangingStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringRangingStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrEnteringFaultedStateEvent(const TwrMgrEnteringFaultedStateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrEnteringFaultedStateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrEnteringFaultedStateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrCannotStartNoTargetsEvent(const TwrMgrCannotStartNoTargetsEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrCannotStartNoTargetsEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrCannotStartNoTargetsEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrFailedToStartInitiatorEvent(const TwrMgrFailedToStartInitiatorEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrFailedToStartInitiatorEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrFailedToStartInitiatorEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrInvalidRangingRateEvent(const TwrMgrInvalidRangingRateEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrInvalidRangingRateEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrInvalidRangingRateEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_TwrMgrRangingRateSetEvent(const TwrMgrRangingRateSetEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, TwrMgrRangingRateSetEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_TwrMgrRangingRateSetEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerTimingMissesEvent(const DataloggerTimingMissesEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerTimingMissesEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerTimingMissesEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_DataloggerLowMemoryEvent(const DataloggerLowMemoryEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, DataloggerLowMemoryEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_DataloggerLowMemoryEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_AppDeviceNotInMappingTableEvent(const AppDeviceNotInMappingTableEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, AppDeviceNotInMappingTableEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppDeviceNotInMappingTableEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_AppUsingDefaultAddressEvent(const AppUsingDefaultAddressEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, AppUsingDefaultAddressEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppUsingDefaultAddressEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_AppFailedToInitDeviceIdEvent(const AppFailedToInitDeviceIdEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, AppFailedToInitDeviceIdEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_AppFailedToInitDeviceIdEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_WatchdogTaskFailureEvent(const WatchdogTaskFailureEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, WatchdogTaskFailureEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_WatchdogTaskFailureEvent, s_tx_buf, stream.bytes_written);
}

void protocol_tx_SystemFatalEvent(const SystemFatalEvent *msg)
{
  pb_ostream_t stream = pb_ostream_from_buffer(s_tx_buf, sizeof(s_tx_buf));
  if (!pb_encode(&stream, SystemFatalEvent_fields, msg))
    return;
  protocol_send_frame(MSG_ID_SystemFatalEvent, s_tx_buf, stream.bytes_written);
}

