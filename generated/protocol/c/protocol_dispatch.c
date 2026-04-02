/* Auto-generated from protocol/protocol.proto barrel by tools/protocol_codegen/codegen_protocol.py */
#include "protocol_dispatch.h"
#include "protocol.pb.h"
#include "protocol_ids.h"
#include <pb_decode.h>
#include <stddef.h>

__attribute__((weak)) void protocol_rx_AckResponse(const AckResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TransportSetRequest(const TransportSetRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TransportGetRequest(const TransportGetRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TransportGetResponse(const TransportGetResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SystemGetUuidRequest(const SystemGetUuidRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SystemGetUuidResponse(const SystemGetUuidResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SystemGetInfoRequest(const SystemGetInfoRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SystemGetInfoResponse(const SystemGetInfoResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_PingRequest(const PingRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_PingResponse(const PingResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SetAddressRequest(const SetAddressRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SetAddressResponse(const SetAddressResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_GetConfigRequest(const GetConfigRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_GetConfigResponse(const GetConfigResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuGetStatusRequest(const ImuGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuGetStatusResponse(const ImuGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuGetDataRequest(const ImuGetDataRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuGetDataResponse(const ImuGetDataResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuStreamStartRequest(const ImuStreamStartRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuStreamStopRequest(const ImuStreamStopRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuCalibrateRequest(const ImuCalibrateRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuCalibrateResponse(const ImuCalibrateResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuCalibrateCompleteEvent(const ImuCalibrateCompleteEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuStreamPayload(const ImuStreamPayload *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeSetTypeRequest(const UwbNodeSetTypeRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetTypeRequest(const UwbNodeGetTypeRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetTypeResponse(const UwbNodeGetTypeResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeSetPositionRequest(const UwbNodeSetPositionRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetPositionRequest(const UwbNodeGetPositionRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetPositionResponse(const UwbNodeGetPositionResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetStatusRequest(const UwbNodeGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeGetStatusResponse(const UwbNodeGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbGetStatusRequest(const UwbGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbGetStatusResponse(const UwbGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbGetStatsRequest(const UwbGetStatsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbGetStatsResponse(const UwbGetStatsResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbStartRequest(const UwbStartRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbStopRequest(const UwbStopRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbResetStatsRequest(const UwbResetStatsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ErrorClearRequest(const ErrorClearRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetTasksRequest(const DataloggerGetTasksRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetTasksResponse(const DataloggerGetTasksResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetStatsRequest(const DataloggerGetStatsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetStatsResponse(const DataloggerGetStatsResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrGetStatusRequest(const TwrGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrGetStatusResponse(const TwrGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrGetResultRequest(const TwrGetResultRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrGetResultResponse(const TwrGetResultResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrRangeRequest(const TwrRangeRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrStartRequest(const TwrMgrStartRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrStopRequest(const TwrMgrStopRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrGetStatusRequest(const TwrMgrGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrGetStatusResponse(const TwrMgrGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrAddTargetRequest(const TwrMgrAddTargetRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrRemoveTargetRequest(const TwrMgrRemoveTargetRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrSetTargetsRequest(const TwrMgrSetTargetsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrClearTargetsRequest(const TwrMgrClearTargetsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrSetRangingRateRequest(const TwrMgrSetRangingRateRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrGetRangingRateRequest(const TwrMgrGetRangingRateRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrGetRangingRateResponse(const TwrMgrGetRangingRateResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_StopwatchGetRequest(const StopwatchGetRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_StopwatchGetResponse(const StopwatchGetResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_StopwatchStartRequest(const StopwatchStartRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_StopwatchStopRequest(const StopwatchStopRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigSendAddressRequest(const OtaConfigSendAddressRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigSendPositionRequest(const OtaConfigSendPositionRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigSendTypeRequest(const OtaConfigSendTypeRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigSendGpioRequest(const OtaConfigSendGpioRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigSetTokenRequest(const OtaConfigSetTokenRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigGetTokenRequest(const OtaConfigGetTokenRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigGetTokenResponse(const OtaConfigGetTokenResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigGetStatsRequest(const OtaConfigGetStatsRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigGetStatsResponse(const OtaConfigGetStatsResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetDebugRequest(const SensorFusionGetDebugRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionSetDebugRequest(const SensorFusionSetDebugRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetStatusRequest(const SensorFusionGetStatusRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetStatusResponse(const SensorFusionGetStatusResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionSetActiveRequest(const SensorFusionSetActiveRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetImuEnabledRequest(const SensorFusionGetImuEnabledRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetImuEnabledResponse(const SensorFusionGetImuEnabledResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionSetImuEnabledRequest(const SensorFusionSetImuEnabledRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionSetNoiseRequest(const SensorFusionSetNoiseRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetNoiseRequest(const SensorFusionGetNoiseRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetNoiseResponse(const SensorFusionGetNoiseResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionGetConfigRequest(const SensorFusionGetConfigRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SensorFusionSetConfigRequest(const SensorFusionSetConfigRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_BeaconPingRequest(const BeaconPingRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_BeaconPingResponse(const BeaconPingResponse *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuInitReturnedNullEvent(const ImuInitReturnedNullEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuProbeInitFailedEvent(const ImuProbeInitFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuInvalidChipIdEvent(const ImuInvalidChipIdEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuAccelConfigFailedEvent(const ImuAccelConfigFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuGyroConfigFailedEvent(const ImuGyroConfigFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuFaultEvent(const ImuFaultEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ImuFailedToPushEventToSensorFusionEvent(const ImuFailedToPushEventToSensorFusionEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigUnknownMessageTypeEvent(const OtaConfigUnknownMessageTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigInvalidAddressEvent(const OtaConfigInvalidAddressEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigAddressChangedEvent(const OtaConfigAddressChangedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigPositionSetEvent(const OtaConfigPositionSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigInvalidNodeTypeEvent(const OtaConfigInvalidNodeTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigNodeTypeSetEvent(const OtaConfigNodeTypeSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigInvalidGpioPinEvent(const OtaConfigInvalidGpioPinEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigInvalidGpioStateEvent(const OtaConfigInvalidGpioStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigGpioSetEvent(const OtaConfigGpioSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigResponseEvent(const OtaConfigResponseEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigAuthFailedEvent(const OtaConfigAuthFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigAckFromEvent(const OtaConfigAckFromEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigNoAckFromEvent(const OtaConfigNoAckFromEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_OtaConfigAuthTokenSetEvent(const OtaConfigAuthTokenSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbFrameTooSmallEvent(const UwbFrameTooSmallEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbPayloadTooLargeEvent(const UwbPayloadTooLargeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbRetryAttemptEvent(const UwbRetryAttemptEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbFaultManualStopRequiredEvent(const UwbFaultManualStopRequiredEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbInvalidProtocolMessageEvent(const UwbInvalidProtocolMessageEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbInvalidProtocolTypeEvent(const UwbInvalidProtocolTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbTxQueueFullEvent(const UwbTxQueueFullEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbProtocolHandlerTableFullEvent(const UwbProtocolHandlerTableFullEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeInvalidTypeEvent(const UwbNodeInvalidTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodeTypeSetEvent(const UwbNodeTypeSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_UwbNodePositionSetEvent(const UwbNodePositionSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrFailedToRegisterProtocolHandlerEvent(const TwrFailedToRegisterProtocolHandlerEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrResponderAutoStartFailedEvent(const TwrResponderAutoStartFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrModuleNotInitializedEvent(const TwrModuleNotInitializedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderFailedToSendMessageEvent(const ResponderFailedToSendMessageEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderPollTooShortEvent(const ResponderPollTooShortEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderFinalTooShortEvent(const ResponderFinalTooShortEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderFailedToSendFinalAckEvent(const ResponderFailedToSendFinalAckEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderUnexpectedMessageTypeEvent(const ResponderUnexpectedMessageTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderTimeoutWaitingForMessageEvent(const ResponderTimeoutWaitingForMessageEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderFaultEvent(const ResponderFaultEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_ResponderCompletedRangingEvent(const ResponderCompletedRangingEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorFailedToSendMessageEvent(const InitiatorFailedToSendMessageEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorResponseTooShortEvent(const InitiatorResponseTooShortEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorFinalAckTooShortEvent(const InitiatorFinalAckTooShortEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorUnexpectedMessageTypeEvent(const InitiatorUnexpectedMessageTypeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorFaultEvent(const InitiatorFaultEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorDistanceCalculationFailedEvent(const InitiatorDistanceCalculationFailedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_InitiatorUwbNotReadyEvent(const InitiatorUwbNotReadyEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSmTxCompletionTimeoutEvent(const TwrSmTxCompletionTimeoutEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSmTxCompleteWrongStateEvent(const TwrSmTxCompleteWrongStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSmTxCompleteMsgIdMismatchEvent(const TwrSmTxCompleteMsgIdMismatchEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSmRxMessageWrongStateEvent(const TwrSmRxMessageWrongStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSmTimeoutUnexpectedStateEvent(const TwrSmTimeoutUnexpectedStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerStrategyNotImplementedEvent(const TwrSchedulerStrategyNotImplementedEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerTargetListFullEvent(const TwrSchedulerTargetListFullEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerTargetAlreadyExistsEvent(const TwrSchedulerTargetAlreadyExistsEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerAddedTargetEvent(const TwrSchedulerAddedTargetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerRemovedTargetEvent(const TwrSchedulerRemovedTargetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerNullAddressesPointerEvent(const TwrSchedulerNullAddressesPointerEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerTargetCountExceedsMaximumEvent(const TwrSchedulerTargetCountExceedsMaximumEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerSetTargetsEvent(const TwrSchedulerSetTargetsEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerClearedAllTargetsEvent(const TwrSchedulerClearedAllTargetsEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerTargetEnabledDisabledEvent(const TwrSchedulerTargetEnabledDisabledEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerTargetBackingOffEvent(const TwrSchedulerTargetBackingOffEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrSchedulerStrategySetEvent(const TwrSchedulerStrategySetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrFailedToPushRangingEventEvent(const TwrMgrFailedToPushRangingEventEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrRangeEvent(const TwrMgrRangeEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrEnteringIdleStateEvent(const TwrMgrEnteringIdleStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrEnteringRangingStateEvent(const TwrMgrEnteringRangingStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrEnteringFaultedStateEvent(const TwrMgrEnteringFaultedStateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrCannotStartNoTargetsEvent(const TwrMgrCannotStartNoTargetsEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrFailedToStartInitiatorEvent(const TwrMgrFailedToStartInitiatorEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrInvalidRangingRateEvent(const TwrMgrInvalidRangingRateEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_TwrMgrRangingRateSetEvent(const TwrMgrRangingRateSetEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerTimingMissesEvent(const DataloggerTimingMissesEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerLowMemoryEvent(const DataloggerLowMemoryEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_AppDeviceNotInMappingTableEvent(const AppDeviceNotInMappingTableEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_AppUsingDefaultAddressEvent(const AppUsingDefaultAddressEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_AppFailedToInitDeviceIdEvent(const AppFailedToInitDeviceIdEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_WatchdogTaskFailureEvent(const WatchdogTaskFailureEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_SystemFatalEvent(const SystemFatalEvent *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetIdleCpuRequest(const DataloggerGetIdleCpuRequest *msg) { (void)msg; }
__attribute__((weak)) void protocol_rx_DataloggerGetIdleCpuResponse(const DataloggerGetIdleCpuResponse *msg) { (void)msg; }

void protocol_dispatch(uint16_t msg_id, const uint8_t *payload, size_t len)
{
  pb_istream_t stream = pb_istream_from_buffer(payload, len);
  switch (msg_id) {
    case MSG_ID_AckResponse: {
      AckResponse decoded = AckResponse_init_zero;
      if (pb_decode(&stream, AckResponse_fields, &decoded))
        protocol_rx_AckResponse(&decoded);
      break;
    }
    case MSG_ID_TransportSetRequest: {
      TransportSetRequest decoded = TransportSetRequest_init_zero;
      if (pb_decode(&stream, TransportSetRequest_fields, &decoded))
        protocol_rx_TransportSetRequest(&decoded);
      break;
    }
    case MSG_ID_TransportGetRequest: {
      TransportGetRequest decoded = TransportGetRequest_init_zero;
      if (pb_decode(&stream, TransportGetRequest_fields, &decoded))
        protocol_rx_TransportGetRequest(&decoded);
      break;
    }
    case MSG_ID_TransportGetResponse: {
      TransportGetResponse decoded = TransportGetResponse_init_zero;
      if (pb_decode(&stream, TransportGetResponse_fields, &decoded))
        protocol_rx_TransportGetResponse(&decoded);
      break;
    }
    case MSG_ID_SystemGetUuidRequest: {
      SystemGetUuidRequest decoded = SystemGetUuidRequest_init_zero;
      if (pb_decode(&stream, SystemGetUuidRequest_fields, &decoded))
        protocol_rx_SystemGetUuidRequest(&decoded);
      break;
    }
    case MSG_ID_SystemGetUuidResponse: {
      SystemGetUuidResponse decoded = SystemGetUuidResponse_init_zero;
      if (pb_decode(&stream, SystemGetUuidResponse_fields, &decoded))
        protocol_rx_SystemGetUuidResponse(&decoded);
      break;
    }
    case MSG_ID_SystemGetInfoRequest: {
      SystemGetInfoRequest decoded = SystemGetInfoRequest_init_zero;
      if (pb_decode(&stream, SystemGetInfoRequest_fields, &decoded))
        protocol_rx_SystemGetInfoRequest(&decoded);
      break;
    }
    case MSG_ID_SystemGetInfoResponse: {
      SystemGetInfoResponse decoded = SystemGetInfoResponse_init_zero;
      if (pb_decode(&stream, SystemGetInfoResponse_fields, &decoded))
        protocol_rx_SystemGetInfoResponse(&decoded);
      break;
    }
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
    case MSG_ID_ImuGetStatusRequest: {
      ImuGetStatusRequest decoded = ImuGetStatusRequest_init_zero;
      if (pb_decode(&stream, ImuGetStatusRequest_fields, &decoded))
        protocol_rx_ImuGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_ImuGetStatusResponse: {
      ImuGetStatusResponse decoded = ImuGetStatusResponse_init_zero;
      if (pb_decode(&stream, ImuGetStatusResponse_fields, &decoded))
        protocol_rx_ImuGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_ImuGetDataRequest: {
      ImuGetDataRequest decoded = ImuGetDataRequest_init_zero;
      if (pb_decode(&stream, ImuGetDataRequest_fields, &decoded))
        protocol_rx_ImuGetDataRequest(&decoded);
      break;
    }
    case MSG_ID_ImuGetDataResponse: {
      ImuGetDataResponse decoded = ImuGetDataResponse_init_zero;
      if (pb_decode(&stream, ImuGetDataResponse_fields, &decoded))
        protocol_rx_ImuGetDataResponse(&decoded);
      break;
    }
    case MSG_ID_ImuStreamStartRequest: {
      ImuStreamStartRequest decoded = ImuStreamStartRequest_init_zero;
      if (pb_decode(&stream, ImuStreamStartRequest_fields, &decoded))
        protocol_rx_ImuStreamStartRequest(&decoded);
      break;
    }
    case MSG_ID_ImuStreamStopRequest: {
      ImuStreamStopRequest decoded = ImuStreamStopRequest_init_zero;
      if (pb_decode(&stream, ImuStreamStopRequest_fields, &decoded))
        protocol_rx_ImuStreamStopRequest(&decoded);
      break;
    }
    case MSG_ID_ImuCalibrateRequest: {
      ImuCalibrateRequest decoded = ImuCalibrateRequest_init_zero;
      if (pb_decode(&stream, ImuCalibrateRequest_fields, &decoded))
        protocol_rx_ImuCalibrateRequest(&decoded);
      break;
    }
    case MSG_ID_ImuCalibrateResponse: {
      ImuCalibrateResponse decoded = ImuCalibrateResponse_init_zero;
      if (pb_decode(&stream, ImuCalibrateResponse_fields, &decoded))
        protocol_rx_ImuCalibrateResponse(&decoded);
      break;
    }
    case MSG_ID_ImuCalibrateCompleteEvent: {
      ImuCalibrateCompleteEvent decoded = ImuCalibrateCompleteEvent_init_zero;
      if (pb_decode(&stream, ImuCalibrateCompleteEvent_fields, &decoded))
        protocol_rx_ImuCalibrateCompleteEvent(&decoded);
      break;
    }
    case MSG_ID_ImuStreamPayload: {
      ImuStreamPayload decoded = ImuStreamPayload_init_zero;
      if (pb_decode(&stream, ImuStreamPayload_fields, &decoded))
        protocol_rx_ImuStreamPayload(&decoded);
      break;
    }
    case MSG_ID_UwbNodeSetTypeRequest: {
      UwbNodeSetTypeRequest decoded = UwbNodeSetTypeRequest_init_zero;
      if (pb_decode(&stream, UwbNodeSetTypeRequest_fields, &decoded))
        protocol_rx_UwbNodeSetTypeRequest(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetTypeRequest: {
      UwbNodeGetTypeRequest decoded = UwbNodeGetTypeRequest_init_zero;
      if (pb_decode(&stream, UwbNodeGetTypeRequest_fields, &decoded))
        protocol_rx_UwbNodeGetTypeRequest(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetTypeResponse: {
      UwbNodeGetTypeResponse decoded = UwbNodeGetTypeResponse_init_zero;
      if (pb_decode(&stream, UwbNodeGetTypeResponse_fields, &decoded))
        protocol_rx_UwbNodeGetTypeResponse(&decoded);
      break;
    }
    case MSG_ID_UwbNodeSetPositionRequest: {
      UwbNodeSetPositionRequest decoded = UwbNodeSetPositionRequest_init_zero;
      if (pb_decode(&stream, UwbNodeSetPositionRequest_fields, &decoded))
        protocol_rx_UwbNodeSetPositionRequest(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetPositionRequest: {
      UwbNodeGetPositionRequest decoded = UwbNodeGetPositionRequest_init_zero;
      if (pb_decode(&stream, UwbNodeGetPositionRequest_fields, &decoded))
        protocol_rx_UwbNodeGetPositionRequest(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetPositionResponse: {
      UwbNodeGetPositionResponse decoded = UwbNodeGetPositionResponse_init_zero;
      if (pb_decode(&stream, UwbNodeGetPositionResponse_fields, &decoded))
        protocol_rx_UwbNodeGetPositionResponse(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetStatusRequest: {
      UwbNodeGetStatusRequest decoded = UwbNodeGetStatusRequest_init_zero;
      if (pb_decode(&stream, UwbNodeGetStatusRequest_fields, &decoded))
        protocol_rx_UwbNodeGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_UwbNodeGetStatusResponse: {
      UwbNodeGetStatusResponse decoded = UwbNodeGetStatusResponse_init_zero;
      if (pb_decode(&stream, UwbNodeGetStatusResponse_fields, &decoded))
        protocol_rx_UwbNodeGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_UwbGetStatusRequest: {
      UwbGetStatusRequest decoded = UwbGetStatusRequest_init_zero;
      if (pb_decode(&stream, UwbGetStatusRequest_fields, &decoded))
        protocol_rx_UwbGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_UwbGetStatusResponse: {
      UwbGetStatusResponse decoded = UwbGetStatusResponse_init_zero;
      if (pb_decode(&stream, UwbGetStatusResponse_fields, &decoded))
        protocol_rx_UwbGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_UwbGetStatsRequest: {
      UwbGetStatsRequest decoded = UwbGetStatsRequest_init_zero;
      if (pb_decode(&stream, UwbGetStatsRequest_fields, &decoded))
        protocol_rx_UwbGetStatsRequest(&decoded);
      break;
    }
    case MSG_ID_UwbGetStatsResponse: {
      UwbGetStatsResponse decoded = UwbGetStatsResponse_init_zero;
      if (pb_decode(&stream, UwbGetStatsResponse_fields, &decoded))
        protocol_rx_UwbGetStatsResponse(&decoded);
      break;
    }
    case MSG_ID_UwbStartRequest: {
      UwbStartRequest decoded = UwbStartRequest_init_zero;
      if (pb_decode(&stream, UwbStartRequest_fields, &decoded))
        protocol_rx_UwbStartRequest(&decoded);
      break;
    }
    case MSG_ID_UwbStopRequest: {
      UwbStopRequest decoded = UwbStopRequest_init_zero;
      if (pb_decode(&stream, UwbStopRequest_fields, &decoded))
        protocol_rx_UwbStopRequest(&decoded);
      break;
    }
    case MSG_ID_UwbResetStatsRequest: {
      UwbResetStatsRequest decoded = UwbResetStatsRequest_init_zero;
      if (pb_decode(&stream, UwbResetStatsRequest_fields, &decoded))
        protocol_rx_UwbResetStatsRequest(&decoded);
      break;
    }
    case MSG_ID_ErrorClearRequest: {
      ErrorClearRequest decoded = ErrorClearRequest_init_zero;
      if (pb_decode(&stream, ErrorClearRequest_fields, &decoded))
        protocol_rx_ErrorClearRequest(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetTasksRequest: {
      DataloggerGetTasksRequest decoded = DataloggerGetTasksRequest_init_zero;
      if (pb_decode(&stream, DataloggerGetTasksRequest_fields, &decoded))
        protocol_rx_DataloggerGetTasksRequest(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetTasksResponse: {
      DataloggerGetTasksResponse decoded = DataloggerGetTasksResponse_init_zero;
      if (pb_decode(&stream, DataloggerGetTasksResponse_fields, &decoded))
        protocol_rx_DataloggerGetTasksResponse(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetStatsRequest: {
      DataloggerGetStatsRequest decoded = DataloggerGetStatsRequest_init_zero;
      if (pb_decode(&stream, DataloggerGetStatsRequest_fields, &decoded))
        protocol_rx_DataloggerGetStatsRequest(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetStatsResponse: {
      DataloggerGetStatsResponse decoded = DataloggerGetStatsResponse_init_zero;
      if (pb_decode(&stream, DataloggerGetStatsResponse_fields, &decoded))
        protocol_rx_DataloggerGetStatsResponse(&decoded);
      break;
    }
    case MSG_ID_TwrGetStatusRequest: {
      TwrGetStatusRequest decoded = TwrGetStatusRequest_init_zero;
      if (pb_decode(&stream, TwrGetStatusRequest_fields, &decoded))
        protocol_rx_TwrGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_TwrGetStatusResponse: {
      TwrGetStatusResponse decoded = TwrGetStatusResponse_init_zero;
      if (pb_decode(&stream, TwrGetStatusResponse_fields, &decoded))
        protocol_rx_TwrGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_TwrGetResultRequest: {
      TwrGetResultRequest decoded = TwrGetResultRequest_init_zero;
      if (pb_decode(&stream, TwrGetResultRequest_fields, &decoded))
        protocol_rx_TwrGetResultRequest(&decoded);
      break;
    }
    case MSG_ID_TwrGetResultResponse: {
      TwrGetResultResponse decoded = TwrGetResultResponse_init_zero;
      if (pb_decode(&stream, TwrGetResultResponse_fields, &decoded))
        protocol_rx_TwrGetResultResponse(&decoded);
      break;
    }
    case MSG_ID_TwrRangeRequest: {
      TwrRangeRequest decoded = TwrRangeRequest_init_zero;
      if (pb_decode(&stream, TwrRangeRequest_fields, &decoded))
        protocol_rx_TwrRangeRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrStartRequest: {
      TwrMgrStartRequest decoded = TwrMgrStartRequest_init_zero;
      if (pb_decode(&stream, TwrMgrStartRequest_fields, &decoded))
        protocol_rx_TwrMgrStartRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrStopRequest: {
      TwrMgrStopRequest decoded = TwrMgrStopRequest_init_zero;
      if (pb_decode(&stream, TwrMgrStopRequest_fields, &decoded))
        protocol_rx_TwrMgrStopRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrGetStatusRequest: {
      TwrMgrGetStatusRequest decoded = TwrMgrGetStatusRequest_init_zero;
      if (pb_decode(&stream, TwrMgrGetStatusRequest_fields, &decoded))
        protocol_rx_TwrMgrGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrGetStatusResponse: {
      TwrMgrGetStatusResponse decoded = TwrMgrGetStatusResponse_init_zero;
      if (pb_decode(&stream, TwrMgrGetStatusResponse_fields, &decoded))
        protocol_rx_TwrMgrGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_TwrMgrAddTargetRequest: {
      TwrMgrAddTargetRequest decoded = TwrMgrAddTargetRequest_init_zero;
      if (pb_decode(&stream, TwrMgrAddTargetRequest_fields, &decoded))
        protocol_rx_TwrMgrAddTargetRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrRemoveTargetRequest: {
      TwrMgrRemoveTargetRequest decoded = TwrMgrRemoveTargetRequest_init_zero;
      if (pb_decode(&stream, TwrMgrRemoveTargetRequest_fields, &decoded))
        protocol_rx_TwrMgrRemoveTargetRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrSetTargetsRequest: {
      TwrMgrSetTargetsRequest decoded = TwrMgrSetTargetsRequest_init_zero;
      if (pb_decode(&stream, TwrMgrSetTargetsRequest_fields, &decoded))
        protocol_rx_TwrMgrSetTargetsRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrClearTargetsRequest: {
      TwrMgrClearTargetsRequest decoded = TwrMgrClearTargetsRequest_init_zero;
      if (pb_decode(&stream, TwrMgrClearTargetsRequest_fields, &decoded))
        protocol_rx_TwrMgrClearTargetsRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrSetRangingRateRequest: {
      TwrMgrSetRangingRateRequest decoded = TwrMgrSetRangingRateRequest_init_zero;
      if (pb_decode(&stream, TwrMgrSetRangingRateRequest_fields, &decoded))
        protocol_rx_TwrMgrSetRangingRateRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrGetRangingRateRequest: {
      TwrMgrGetRangingRateRequest decoded = TwrMgrGetRangingRateRequest_init_zero;
      if (pb_decode(&stream, TwrMgrGetRangingRateRequest_fields, &decoded))
        protocol_rx_TwrMgrGetRangingRateRequest(&decoded);
      break;
    }
    case MSG_ID_TwrMgrGetRangingRateResponse: {
      TwrMgrGetRangingRateResponse decoded = TwrMgrGetRangingRateResponse_init_zero;
      if (pb_decode(&stream, TwrMgrGetRangingRateResponse_fields, &decoded))
        protocol_rx_TwrMgrGetRangingRateResponse(&decoded);
      break;
    }
    case MSG_ID_StopwatchGetRequest: {
      StopwatchGetRequest decoded = StopwatchGetRequest_init_zero;
      if (pb_decode(&stream, StopwatchGetRequest_fields, &decoded))
        protocol_rx_StopwatchGetRequest(&decoded);
      break;
    }
    case MSG_ID_StopwatchGetResponse: {
      StopwatchGetResponse decoded = StopwatchGetResponse_init_zero;
      if (pb_decode(&stream, StopwatchGetResponse_fields, &decoded))
        protocol_rx_StopwatchGetResponse(&decoded);
      break;
    }
    case MSG_ID_StopwatchStartRequest: {
      StopwatchStartRequest decoded = StopwatchStartRequest_init_zero;
      if (pb_decode(&stream, StopwatchStartRequest_fields, &decoded))
        protocol_rx_StopwatchStartRequest(&decoded);
      break;
    }
    case MSG_ID_StopwatchStopRequest: {
      StopwatchStopRequest decoded = StopwatchStopRequest_init_zero;
      if (pb_decode(&stream, StopwatchStopRequest_fields, &decoded))
        protocol_rx_StopwatchStopRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigSendAddressRequest: {
      OtaConfigSendAddressRequest decoded = OtaConfigSendAddressRequest_init_zero;
      if (pb_decode(&stream, OtaConfigSendAddressRequest_fields, &decoded))
        protocol_rx_OtaConfigSendAddressRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigSendPositionRequest: {
      OtaConfigSendPositionRequest decoded = OtaConfigSendPositionRequest_init_zero;
      if (pb_decode(&stream, OtaConfigSendPositionRequest_fields, &decoded))
        protocol_rx_OtaConfigSendPositionRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigSendTypeRequest: {
      OtaConfigSendTypeRequest decoded = OtaConfigSendTypeRequest_init_zero;
      if (pb_decode(&stream, OtaConfigSendTypeRequest_fields, &decoded))
        protocol_rx_OtaConfigSendTypeRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigSendGpioRequest: {
      OtaConfigSendGpioRequest decoded = OtaConfigSendGpioRequest_init_zero;
      if (pb_decode(&stream, OtaConfigSendGpioRequest_fields, &decoded))
        protocol_rx_OtaConfigSendGpioRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigSetTokenRequest: {
      OtaConfigSetTokenRequest decoded = OtaConfigSetTokenRequest_init_zero;
      if (pb_decode(&stream, OtaConfigSetTokenRequest_fields, &decoded))
        protocol_rx_OtaConfigSetTokenRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigGetTokenRequest: {
      OtaConfigGetTokenRequest decoded = OtaConfigGetTokenRequest_init_zero;
      if (pb_decode(&stream, OtaConfigGetTokenRequest_fields, &decoded))
        protocol_rx_OtaConfigGetTokenRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigGetTokenResponse: {
      OtaConfigGetTokenResponse decoded = OtaConfigGetTokenResponse_init_zero;
      if (pb_decode(&stream, OtaConfigGetTokenResponse_fields, &decoded))
        protocol_rx_OtaConfigGetTokenResponse(&decoded);
      break;
    }
    case MSG_ID_OtaConfigGetStatsRequest: {
      OtaConfigGetStatsRequest decoded = OtaConfigGetStatsRequest_init_zero;
      if (pb_decode(&stream, OtaConfigGetStatsRequest_fields, &decoded))
        protocol_rx_OtaConfigGetStatsRequest(&decoded);
      break;
    }
    case MSG_ID_OtaConfigGetStatsResponse: {
      OtaConfigGetStatsResponse decoded = OtaConfigGetStatsResponse_init_zero;
      if (pb_decode(&stream, OtaConfigGetStatsResponse_fields, &decoded))
        protocol_rx_OtaConfigGetStatsResponse(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetDebugRequest: {
      SensorFusionGetDebugRequest decoded = SensorFusionGetDebugRequest_init_zero;
      if (pb_decode(&stream, SensorFusionGetDebugRequest_fields, &decoded))
        protocol_rx_SensorFusionGetDebugRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionSetDebugRequest: {
      SensorFusionSetDebugRequest decoded = SensorFusionSetDebugRequest_init_zero;
      if (pb_decode(&stream, SensorFusionSetDebugRequest_fields, &decoded))
        protocol_rx_SensorFusionSetDebugRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetStatusRequest: {
      SensorFusionGetStatusRequest decoded = SensorFusionGetStatusRequest_init_zero;
      if (pb_decode(&stream, SensorFusionGetStatusRequest_fields, &decoded))
        protocol_rx_SensorFusionGetStatusRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetStatusResponse: {
      SensorFusionGetStatusResponse decoded = SensorFusionGetStatusResponse_init_zero;
      if (pb_decode(&stream, SensorFusionGetStatusResponse_fields, &decoded))
        protocol_rx_SensorFusionGetStatusResponse(&decoded);
      break;
    }
    case MSG_ID_SensorFusionSetActiveRequest: {
      SensorFusionSetActiveRequest decoded = SensorFusionSetActiveRequest_init_zero;
      if (pb_decode(&stream, SensorFusionSetActiveRequest_fields, &decoded))
        protocol_rx_SensorFusionSetActiveRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetImuEnabledRequest: {
      SensorFusionGetImuEnabledRequest decoded = SensorFusionGetImuEnabledRequest_init_zero;
      if (pb_decode(&stream, SensorFusionGetImuEnabledRequest_fields, &decoded))
        protocol_rx_SensorFusionGetImuEnabledRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetImuEnabledResponse: {
      SensorFusionGetImuEnabledResponse decoded = SensorFusionGetImuEnabledResponse_init_zero;
      if (pb_decode(&stream, SensorFusionGetImuEnabledResponse_fields, &decoded))
        protocol_rx_SensorFusionGetImuEnabledResponse(&decoded);
      break;
    }
    case MSG_ID_SensorFusionSetImuEnabledRequest: {
      SensorFusionSetImuEnabledRequest decoded = SensorFusionSetImuEnabledRequest_init_zero;
      if (pb_decode(&stream, SensorFusionSetImuEnabledRequest_fields, &decoded))
        protocol_rx_SensorFusionSetImuEnabledRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionSetNoiseRequest: {
      SensorFusionSetNoiseRequest decoded = SensorFusionSetNoiseRequest_init_zero;
      if (pb_decode(&stream, SensorFusionSetNoiseRequest_fields, &decoded))
        protocol_rx_SensorFusionSetNoiseRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetNoiseRequest: {
      SensorFusionGetNoiseRequest decoded = SensorFusionGetNoiseRequest_init_zero;
      if (pb_decode(&stream, SensorFusionGetNoiseRequest_fields, &decoded))
        protocol_rx_SensorFusionGetNoiseRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetNoiseResponse: {
      SensorFusionGetNoiseResponse decoded = SensorFusionGetNoiseResponse_init_zero;
      if (pb_decode(&stream, SensorFusionGetNoiseResponse_fields, &decoded))
        protocol_rx_SensorFusionGetNoiseResponse(&decoded);
      break;
    }
    case MSG_ID_SensorFusionGetConfigRequest: {
      SensorFusionGetConfigRequest decoded = SensorFusionGetConfigRequest_init_zero;
      if (pb_decode(&stream, SensorFusionGetConfigRequest_fields, &decoded))
        protocol_rx_SensorFusionGetConfigRequest(&decoded);
      break;
    }
    case MSG_ID_SensorFusionSetConfigRequest: {
      SensorFusionSetConfigRequest decoded = SensorFusionSetConfigRequest_init_zero;
      if (pb_decode(&stream, SensorFusionSetConfigRequest_fields, &decoded))
        protocol_rx_SensorFusionSetConfigRequest(&decoded);
      break;
    }
    case MSG_ID_BeaconPingRequest: {
      BeaconPingRequest decoded = BeaconPingRequest_init_zero;
      if (pb_decode(&stream, BeaconPingRequest_fields, &decoded))
        protocol_rx_BeaconPingRequest(&decoded);
      break;
    }
    case MSG_ID_BeaconPingResponse: {
      BeaconPingResponse decoded = BeaconPingResponse_init_zero;
      if (pb_decode(&stream, BeaconPingResponse_fields, &decoded))
        protocol_rx_BeaconPingResponse(&decoded);
      break;
    }
    case MSG_ID_ImuInitReturnedNullEvent: {
      ImuInitReturnedNullEvent decoded = ImuInitReturnedNullEvent_init_zero;
      if (pb_decode(&stream, ImuInitReturnedNullEvent_fields, &decoded))
        protocol_rx_ImuInitReturnedNullEvent(&decoded);
      break;
    }
    case MSG_ID_ImuProbeInitFailedEvent: {
      ImuProbeInitFailedEvent decoded = ImuProbeInitFailedEvent_init_zero;
      if (pb_decode(&stream, ImuProbeInitFailedEvent_fields, &decoded))
        protocol_rx_ImuProbeInitFailedEvent(&decoded);
      break;
    }
    case MSG_ID_ImuInvalidChipIdEvent: {
      ImuInvalidChipIdEvent decoded = ImuInvalidChipIdEvent_init_zero;
      if (pb_decode(&stream, ImuInvalidChipIdEvent_fields, &decoded))
        protocol_rx_ImuInvalidChipIdEvent(&decoded);
      break;
    }
    case MSG_ID_ImuAccelConfigFailedEvent: {
      ImuAccelConfigFailedEvent decoded = ImuAccelConfigFailedEvent_init_zero;
      if (pb_decode(&stream, ImuAccelConfigFailedEvent_fields, &decoded))
        protocol_rx_ImuAccelConfigFailedEvent(&decoded);
      break;
    }
    case MSG_ID_ImuGyroConfigFailedEvent: {
      ImuGyroConfigFailedEvent decoded = ImuGyroConfigFailedEvent_init_zero;
      if (pb_decode(&stream, ImuGyroConfigFailedEvent_fields, &decoded))
        protocol_rx_ImuGyroConfigFailedEvent(&decoded);
      break;
    }
    case MSG_ID_ImuFaultEvent: {
      ImuFaultEvent decoded = ImuFaultEvent_init_zero;
      if (pb_decode(&stream, ImuFaultEvent_fields, &decoded))
        protocol_rx_ImuFaultEvent(&decoded);
      break;
    }
    case MSG_ID_ImuFailedToPushEventToSensorFusionEvent: {
      ImuFailedToPushEventToSensorFusionEvent decoded = ImuFailedToPushEventToSensorFusionEvent_init_zero;
      if (pb_decode(&stream, ImuFailedToPushEventToSensorFusionEvent_fields, &decoded))
        protocol_rx_ImuFailedToPushEventToSensorFusionEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigUnknownMessageTypeEvent: {
      OtaConfigUnknownMessageTypeEvent decoded = OtaConfigUnknownMessageTypeEvent_init_zero;
      if (pb_decode(&stream, OtaConfigUnknownMessageTypeEvent_fields, &decoded))
        protocol_rx_OtaConfigUnknownMessageTypeEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigInvalidAddressEvent: {
      OtaConfigInvalidAddressEvent decoded = OtaConfigInvalidAddressEvent_init_zero;
      if (pb_decode(&stream, OtaConfigInvalidAddressEvent_fields, &decoded))
        protocol_rx_OtaConfigInvalidAddressEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigAddressChangedEvent: {
      OtaConfigAddressChangedEvent decoded = OtaConfigAddressChangedEvent_init_zero;
      if (pb_decode(&stream, OtaConfigAddressChangedEvent_fields, &decoded))
        protocol_rx_OtaConfigAddressChangedEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigPositionSetEvent: {
      OtaConfigPositionSetEvent decoded = OtaConfigPositionSetEvent_init_zero;
      if (pb_decode(&stream, OtaConfigPositionSetEvent_fields, &decoded))
        protocol_rx_OtaConfigPositionSetEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigInvalidNodeTypeEvent: {
      OtaConfigInvalidNodeTypeEvent decoded = OtaConfigInvalidNodeTypeEvent_init_zero;
      if (pb_decode(&stream, OtaConfigInvalidNodeTypeEvent_fields, &decoded))
        protocol_rx_OtaConfigInvalidNodeTypeEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigNodeTypeSetEvent: {
      OtaConfigNodeTypeSetEvent decoded = OtaConfigNodeTypeSetEvent_init_zero;
      if (pb_decode(&stream, OtaConfigNodeTypeSetEvent_fields, &decoded))
        protocol_rx_OtaConfigNodeTypeSetEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigInvalidGpioPinEvent: {
      OtaConfigInvalidGpioPinEvent decoded = OtaConfigInvalidGpioPinEvent_init_zero;
      if (pb_decode(&stream, OtaConfigInvalidGpioPinEvent_fields, &decoded))
        protocol_rx_OtaConfigInvalidGpioPinEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigInvalidGpioStateEvent: {
      OtaConfigInvalidGpioStateEvent decoded = OtaConfigInvalidGpioStateEvent_init_zero;
      if (pb_decode(&stream, OtaConfigInvalidGpioStateEvent_fields, &decoded))
        protocol_rx_OtaConfigInvalidGpioStateEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigGpioSetEvent: {
      OtaConfigGpioSetEvent decoded = OtaConfigGpioSetEvent_init_zero;
      if (pb_decode(&stream, OtaConfigGpioSetEvent_fields, &decoded))
        protocol_rx_OtaConfigGpioSetEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigResponseEvent: {
      OtaConfigResponseEvent decoded = OtaConfigResponseEvent_init_zero;
      if (pb_decode(&stream, OtaConfigResponseEvent_fields, &decoded))
        protocol_rx_OtaConfigResponseEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigAuthFailedEvent: {
      OtaConfigAuthFailedEvent decoded = OtaConfigAuthFailedEvent_init_zero;
      if (pb_decode(&stream, OtaConfigAuthFailedEvent_fields, &decoded))
        protocol_rx_OtaConfigAuthFailedEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigAckFromEvent: {
      OtaConfigAckFromEvent decoded = OtaConfigAckFromEvent_init_zero;
      if (pb_decode(&stream, OtaConfigAckFromEvent_fields, &decoded))
        protocol_rx_OtaConfigAckFromEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigNoAckFromEvent: {
      OtaConfigNoAckFromEvent decoded = OtaConfigNoAckFromEvent_init_zero;
      if (pb_decode(&stream, OtaConfigNoAckFromEvent_fields, &decoded))
        protocol_rx_OtaConfigNoAckFromEvent(&decoded);
      break;
    }
    case MSG_ID_OtaConfigAuthTokenSetEvent: {
      OtaConfigAuthTokenSetEvent decoded = OtaConfigAuthTokenSetEvent_init_zero;
      if (pb_decode(&stream, OtaConfigAuthTokenSetEvent_fields, &decoded))
        protocol_rx_OtaConfigAuthTokenSetEvent(&decoded);
      break;
    }
    case MSG_ID_UwbFrameTooSmallEvent: {
      UwbFrameTooSmallEvent decoded = UwbFrameTooSmallEvent_init_zero;
      if (pb_decode(&stream, UwbFrameTooSmallEvent_fields, &decoded))
        protocol_rx_UwbFrameTooSmallEvent(&decoded);
      break;
    }
    case MSG_ID_UwbPayloadTooLargeEvent: {
      UwbPayloadTooLargeEvent decoded = UwbPayloadTooLargeEvent_init_zero;
      if (pb_decode(&stream, UwbPayloadTooLargeEvent_fields, &decoded))
        protocol_rx_UwbPayloadTooLargeEvent(&decoded);
      break;
    }
    case MSG_ID_UwbRetryAttemptEvent: {
      UwbRetryAttemptEvent decoded = UwbRetryAttemptEvent_init_zero;
      if (pb_decode(&stream, UwbRetryAttemptEvent_fields, &decoded))
        protocol_rx_UwbRetryAttemptEvent(&decoded);
      break;
    }
    case MSG_ID_UwbFaultManualStopRequiredEvent: {
      UwbFaultManualStopRequiredEvent decoded = UwbFaultManualStopRequiredEvent_init_zero;
      if (pb_decode(&stream, UwbFaultManualStopRequiredEvent_fields, &decoded))
        protocol_rx_UwbFaultManualStopRequiredEvent(&decoded);
      break;
    }
    case MSG_ID_UwbInvalidProtocolMessageEvent: {
      UwbInvalidProtocolMessageEvent decoded = UwbInvalidProtocolMessageEvent_init_zero;
      if (pb_decode(&stream, UwbInvalidProtocolMessageEvent_fields, &decoded))
        protocol_rx_UwbInvalidProtocolMessageEvent(&decoded);
      break;
    }
    case MSG_ID_UwbInvalidProtocolTypeEvent: {
      UwbInvalidProtocolTypeEvent decoded = UwbInvalidProtocolTypeEvent_init_zero;
      if (pb_decode(&stream, UwbInvalidProtocolTypeEvent_fields, &decoded))
        protocol_rx_UwbInvalidProtocolTypeEvent(&decoded);
      break;
    }
    case MSG_ID_UwbTxQueueFullEvent: {
      UwbTxQueueFullEvent decoded = UwbTxQueueFullEvent_init_zero;
      if (pb_decode(&stream, UwbTxQueueFullEvent_fields, &decoded))
        protocol_rx_UwbTxQueueFullEvent(&decoded);
      break;
    }
    case MSG_ID_UwbProtocolHandlerTableFullEvent: {
      UwbProtocolHandlerTableFullEvent decoded = UwbProtocolHandlerTableFullEvent_init_zero;
      if (pb_decode(&stream, UwbProtocolHandlerTableFullEvent_fields, &decoded))
        protocol_rx_UwbProtocolHandlerTableFullEvent(&decoded);
      break;
    }
    case MSG_ID_UwbNodeInvalidTypeEvent: {
      UwbNodeInvalidTypeEvent decoded = UwbNodeInvalidTypeEvent_init_zero;
      if (pb_decode(&stream, UwbNodeInvalidTypeEvent_fields, &decoded))
        protocol_rx_UwbNodeInvalidTypeEvent(&decoded);
      break;
    }
    case MSG_ID_UwbNodeTypeSetEvent: {
      UwbNodeTypeSetEvent decoded = UwbNodeTypeSetEvent_init_zero;
      if (pb_decode(&stream, UwbNodeTypeSetEvent_fields, &decoded))
        protocol_rx_UwbNodeTypeSetEvent(&decoded);
      break;
    }
    case MSG_ID_UwbNodePositionSetEvent: {
      UwbNodePositionSetEvent decoded = UwbNodePositionSetEvent_init_zero;
      if (pb_decode(&stream, UwbNodePositionSetEvent_fields, &decoded))
        protocol_rx_UwbNodePositionSetEvent(&decoded);
      break;
    }
    case MSG_ID_TwrFailedToRegisterProtocolHandlerEvent: {
      TwrFailedToRegisterProtocolHandlerEvent decoded = TwrFailedToRegisterProtocolHandlerEvent_init_zero;
      if (pb_decode(&stream, TwrFailedToRegisterProtocolHandlerEvent_fields, &decoded))
        protocol_rx_TwrFailedToRegisterProtocolHandlerEvent(&decoded);
      break;
    }
    case MSG_ID_TwrResponderAutoStartFailedEvent: {
      TwrResponderAutoStartFailedEvent decoded = TwrResponderAutoStartFailedEvent_init_zero;
      if (pb_decode(&stream, TwrResponderAutoStartFailedEvent_fields, &decoded))
        protocol_rx_TwrResponderAutoStartFailedEvent(&decoded);
      break;
    }
    case MSG_ID_TwrModuleNotInitializedEvent: {
      TwrModuleNotInitializedEvent decoded = TwrModuleNotInitializedEvent_init_zero;
      if (pb_decode(&stream, TwrModuleNotInitializedEvent_fields, &decoded))
        protocol_rx_TwrModuleNotInitializedEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderFailedToSendMessageEvent: {
      ResponderFailedToSendMessageEvent decoded = ResponderFailedToSendMessageEvent_init_zero;
      if (pb_decode(&stream, ResponderFailedToSendMessageEvent_fields, &decoded))
        protocol_rx_ResponderFailedToSendMessageEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderPollTooShortEvent: {
      ResponderPollTooShortEvent decoded = ResponderPollTooShortEvent_init_zero;
      if (pb_decode(&stream, ResponderPollTooShortEvent_fields, &decoded))
        protocol_rx_ResponderPollTooShortEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderFinalTooShortEvent: {
      ResponderFinalTooShortEvent decoded = ResponderFinalTooShortEvent_init_zero;
      if (pb_decode(&stream, ResponderFinalTooShortEvent_fields, &decoded))
        protocol_rx_ResponderFinalTooShortEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderFailedToSendFinalAckEvent: {
      ResponderFailedToSendFinalAckEvent decoded = ResponderFailedToSendFinalAckEvent_init_zero;
      if (pb_decode(&stream, ResponderFailedToSendFinalAckEvent_fields, &decoded))
        protocol_rx_ResponderFailedToSendFinalAckEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderUnexpectedMessageTypeEvent: {
      ResponderUnexpectedMessageTypeEvent decoded = ResponderUnexpectedMessageTypeEvent_init_zero;
      if (pb_decode(&stream, ResponderUnexpectedMessageTypeEvent_fields, &decoded))
        protocol_rx_ResponderUnexpectedMessageTypeEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderTimeoutWaitingForMessageEvent: {
      ResponderTimeoutWaitingForMessageEvent decoded = ResponderTimeoutWaitingForMessageEvent_init_zero;
      if (pb_decode(&stream, ResponderTimeoutWaitingForMessageEvent_fields, &decoded))
        protocol_rx_ResponderTimeoutWaitingForMessageEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderFaultEvent: {
      ResponderFaultEvent decoded = ResponderFaultEvent_init_zero;
      if (pb_decode(&stream, ResponderFaultEvent_fields, &decoded))
        protocol_rx_ResponderFaultEvent(&decoded);
      break;
    }
    case MSG_ID_ResponderCompletedRangingEvent: {
      ResponderCompletedRangingEvent decoded = ResponderCompletedRangingEvent_init_zero;
      if (pb_decode(&stream, ResponderCompletedRangingEvent_fields, &decoded))
        protocol_rx_ResponderCompletedRangingEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorFailedToSendMessageEvent: {
      InitiatorFailedToSendMessageEvent decoded = InitiatorFailedToSendMessageEvent_init_zero;
      if (pb_decode(&stream, InitiatorFailedToSendMessageEvent_fields, &decoded))
        protocol_rx_InitiatorFailedToSendMessageEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorResponseTooShortEvent: {
      InitiatorResponseTooShortEvent decoded = InitiatorResponseTooShortEvent_init_zero;
      if (pb_decode(&stream, InitiatorResponseTooShortEvent_fields, &decoded))
        protocol_rx_InitiatorResponseTooShortEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorFinalAckTooShortEvent: {
      InitiatorFinalAckTooShortEvent decoded = InitiatorFinalAckTooShortEvent_init_zero;
      if (pb_decode(&stream, InitiatorFinalAckTooShortEvent_fields, &decoded))
        protocol_rx_InitiatorFinalAckTooShortEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorUnexpectedMessageTypeEvent: {
      InitiatorUnexpectedMessageTypeEvent decoded = InitiatorUnexpectedMessageTypeEvent_init_zero;
      if (pb_decode(&stream, InitiatorUnexpectedMessageTypeEvent_fields, &decoded))
        protocol_rx_InitiatorUnexpectedMessageTypeEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorFaultEvent: {
      InitiatorFaultEvent decoded = InitiatorFaultEvent_init_zero;
      if (pb_decode(&stream, InitiatorFaultEvent_fields, &decoded))
        protocol_rx_InitiatorFaultEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorDistanceCalculationFailedEvent: {
      InitiatorDistanceCalculationFailedEvent decoded = InitiatorDistanceCalculationFailedEvent_init_zero;
      if (pb_decode(&stream, InitiatorDistanceCalculationFailedEvent_fields, &decoded))
        protocol_rx_InitiatorDistanceCalculationFailedEvent(&decoded);
      break;
    }
    case MSG_ID_InitiatorUwbNotReadyEvent: {
      InitiatorUwbNotReadyEvent decoded = InitiatorUwbNotReadyEvent_init_zero;
      if (pb_decode(&stream, InitiatorUwbNotReadyEvent_fields, &decoded))
        protocol_rx_InitiatorUwbNotReadyEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSmTxCompletionTimeoutEvent: {
      TwrSmTxCompletionTimeoutEvent decoded = TwrSmTxCompletionTimeoutEvent_init_zero;
      if (pb_decode(&stream, TwrSmTxCompletionTimeoutEvent_fields, &decoded))
        protocol_rx_TwrSmTxCompletionTimeoutEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSmTxCompleteWrongStateEvent: {
      TwrSmTxCompleteWrongStateEvent decoded = TwrSmTxCompleteWrongStateEvent_init_zero;
      if (pb_decode(&stream, TwrSmTxCompleteWrongStateEvent_fields, &decoded))
        protocol_rx_TwrSmTxCompleteWrongStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSmTxCompleteMsgIdMismatchEvent: {
      TwrSmTxCompleteMsgIdMismatchEvent decoded = TwrSmTxCompleteMsgIdMismatchEvent_init_zero;
      if (pb_decode(&stream, TwrSmTxCompleteMsgIdMismatchEvent_fields, &decoded))
        protocol_rx_TwrSmTxCompleteMsgIdMismatchEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSmRxMessageWrongStateEvent: {
      TwrSmRxMessageWrongStateEvent decoded = TwrSmRxMessageWrongStateEvent_init_zero;
      if (pb_decode(&stream, TwrSmRxMessageWrongStateEvent_fields, &decoded))
        protocol_rx_TwrSmRxMessageWrongStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSmTimeoutUnexpectedStateEvent: {
      TwrSmTimeoutUnexpectedStateEvent decoded = TwrSmTimeoutUnexpectedStateEvent_init_zero;
      if (pb_decode(&stream, TwrSmTimeoutUnexpectedStateEvent_fields, &decoded))
        protocol_rx_TwrSmTimeoutUnexpectedStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerStrategyNotImplementedEvent: {
      TwrSchedulerStrategyNotImplementedEvent decoded = TwrSchedulerStrategyNotImplementedEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerStrategyNotImplementedEvent_fields, &decoded))
        protocol_rx_TwrSchedulerStrategyNotImplementedEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerTargetListFullEvent: {
      TwrSchedulerTargetListFullEvent decoded = TwrSchedulerTargetListFullEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerTargetListFullEvent_fields, &decoded))
        protocol_rx_TwrSchedulerTargetListFullEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerTargetAlreadyExistsEvent: {
      TwrSchedulerTargetAlreadyExistsEvent decoded = TwrSchedulerTargetAlreadyExistsEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerTargetAlreadyExistsEvent_fields, &decoded))
        protocol_rx_TwrSchedulerTargetAlreadyExistsEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerAddedTargetEvent: {
      TwrSchedulerAddedTargetEvent decoded = TwrSchedulerAddedTargetEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerAddedTargetEvent_fields, &decoded))
        protocol_rx_TwrSchedulerAddedTargetEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerRemovedTargetEvent: {
      TwrSchedulerRemovedTargetEvent decoded = TwrSchedulerRemovedTargetEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerRemovedTargetEvent_fields, &decoded))
        protocol_rx_TwrSchedulerRemovedTargetEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerNullAddressesPointerEvent: {
      TwrSchedulerNullAddressesPointerEvent decoded = TwrSchedulerNullAddressesPointerEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerNullAddressesPointerEvent_fields, &decoded))
        protocol_rx_TwrSchedulerNullAddressesPointerEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerTargetCountExceedsMaximumEvent: {
      TwrSchedulerTargetCountExceedsMaximumEvent decoded = TwrSchedulerTargetCountExceedsMaximumEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerTargetCountExceedsMaximumEvent_fields, &decoded))
        protocol_rx_TwrSchedulerTargetCountExceedsMaximumEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerSetTargetsEvent: {
      TwrSchedulerSetTargetsEvent decoded = TwrSchedulerSetTargetsEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerSetTargetsEvent_fields, &decoded))
        protocol_rx_TwrSchedulerSetTargetsEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerClearedAllTargetsEvent: {
      TwrSchedulerClearedAllTargetsEvent decoded = TwrSchedulerClearedAllTargetsEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerClearedAllTargetsEvent_fields, &decoded))
        protocol_rx_TwrSchedulerClearedAllTargetsEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerTargetEnabledDisabledEvent: {
      TwrSchedulerTargetEnabledDisabledEvent decoded = TwrSchedulerTargetEnabledDisabledEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerTargetEnabledDisabledEvent_fields, &decoded))
        protocol_rx_TwrSchedulerTargetEnabledDisabledEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerTargetBackingOffEvent: {
      TwrSchedulerTargetBackingOffEvent decoded = TwrSchedulerTargetBackingOffEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerTargetBackingOffEvent_fields, &decoded))
        protocol_rx_TwrSchedulerTargetBackingOffEvent(&decoded);
      break;
    }
    case MSG_ID_TwrSchedulerStrategySetEvent: {
      TwrSchedulerStrategySetEvent decoded = TwrSchedulerStrategySetEvent_init_zero;
      if (pb_decode(&stream, TwrSchedulerStrategySetEvent_fields, &decoded))
        protocol_rx_TwrSchedulerStrategySetEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrFailedToPushRangingEventEvent: {
      TwrMgrFailedToPushRangingEventEvent decoded = TwrMgrFailedToPushRangingEventEvent_init_zero;
      if (pb_decode(&stream, TwrMgrFailedToPushRangingEventEvent_fields, &decoded))
        protocol_rx_TwrMgrFailedToPushRangingEventEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrRangeEvent: {
      TwrMgrRangeEvent decoded = TwrMgrRangeEvent_init_zero;
      if (pb_decode(&stream, TwrMgrRangeEvent_fields, &decoded))
        protocol_rx_TwrMgrRangeEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrEnteringIdleStateEvent: {
      TwrMgrEnteringIdleStateEvent decoded = TwrMgrEnteringIdleStateEvent_init_zero;
      if (pb_decode(&stream, TwrMgrEnteringIdleStateEvent_fields, &decoded))
        protocol_rx_TwrMgrEnteringIdleStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrEnteringRangingStateEvent: {
      TwrMgrEnteringRangingStateEvent decoded = TwrMgrEnteringRangingStateEvent_init_zero;
      if (pb_decode(&stream, TwrMgrEnteringRangingStateEvent_fields, &decoded))
        protocol_rx_TwrMgrEnteringRangingStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrEnteringFaultedStateEvent: {
      TwrMgrEnteringFaultedStateEvent decoded = TwrMgrEnteringFaultedStateEvent_init_zero;
      if (pb_decode(&stream, TwrMgrEnteringFaultedStateEvent_fields, &decoded))
        protocol_rx_TwrMgrEnteringFaultedStateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrCannotStartNoTargetsEvent: {
      TwrMgrCannotStartNoTargetsEvent decoded = TwrMgrCannotStartNoTargetsEvent_init_zero;
      if (pb_decode(&stream, TwrMgrCannotStartNoTargetsEvent_fields, &decoded))
        protocol_rx_TwrMgrCannotStartNoTargetsEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrFailedToStartInitiatorEvent: {
      TwrMgrFailedToStartInitiatorEvent decoded = TwrMgrFailedToStartInitiatorEvent_init_zero;
      if (pb_decode(&stream, TwrMgrFailedToStartInitiatorEvent_fields, &decoded))
        protocol_rx_TwrMgrFailedToStartInitiatorEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrInvalidRangingRateEvent: {
      TwrMgrInvalidRangingRateEvent decoded = TwrMgrInvalidRangingRateEvent_init_zero;
      if (pb_decode(&stream, TwrMgrInvalidRangingRateEvent_fields, &decoded))
        protocol_rx_TwrMgrInvalidRangingRateEvent(&decoded);
      break;
    }
    case MSG_ID_TwrMgrRangingRateSetEvent: {
      TwrMgrRangingRateSetEvent decoded = TwrMgrRangingRateSetEvent_init_zero;
      if (pb_decode(&stream, TwrMgrRangingRateSetEvent_fields, &decoded))
        protocol_rx_TwrMgrRangingRateSetEvent(&decoded);
      break;
    }
    case MSG_ID_DataloggerTimingMissesEvent: {
      DataloggerTimingMissesEvent decoded = DataloggerTimingMissesEvent_init_zero;
      if (pb_decode(&stream, DataloggerTimingMissesEvent_fields, &decoded))
        protocol_rx_DataloggerTimingMissesEvent(&decoded);
      break;
    }
    case MSG_ID_DataloggerLowMemoryEvent: {
      DataloggerLowMemoryEvent decoded = DataloggerLowMemoryEvent_init_zero;
      if (pb_decode(&stream, DataloggerLowMemoryEvent_fields, &decoded))
        protocol_rx_DataloggerLowMemoryEvent(&decoded);
      break;
    }
    case MSG_ID_AppDeviceNotInMappingTableEvent: {
      AppDeviceNotInMappingTableEvent decoded = AppDeviceNotInMappingTableEvent_init_zero;
      if (pb_decode(&stream, AppDeviceNotInMappingTableEvent_fields, &decoded))
        protocol_rx_AppDeviceNotInMappingTableEvent(&decoded);
      break;
    }
    case MSG_ID_AppUsingDefaultAddressEvent: {
      AppUsingDefaultAddressEvent decoded = AppUsingDefaultAddressEvent_init_zero;
      if (pb_decode(&stream, AppUsingDefaultAddressEvent_fields, &decoded))
        protocol_rx_AppUsingDefaultAddressEvent(&decoded);
      break;
    }
    case MSG_ID_AppFailedToInitDeviceIdEvent: {
      AppFailedToInitDeviceIdEvent decoded = AppFailedToInitDeviceIdEvent_init_zero;
      if (pb_decode(&stream, AppFailedToInitDeviceIdEvent_fields, &decoded))
        protocol_rx_AppFailedToInitDeviceIdEvent(&decoded);
      break;
    }
    case MSG_ID_WatchdogTaskFailureEvent: {
      WatchdogTaskFailureEvent decoded = WatchdogTaskFailureEvent_init_zero;
      if (pb_decode(&stream, WatchdogTaskFailureEvent_fields, &decoded))
        protocol_rx_WatchdogTaskFailureEvent(&decoded);
      break;
    }
    case MSG_ID_SystemFatalEvent: {
      SystemFatalEvent decoded = SystemFatalEvent_init_zero;
      if (pb_decode(&stream, SystemFatalEvent_fields, &decoded))
        protocol_rx_SystemFatalEvent(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetIdleCpuRequest: {
      DataloggerGetIdleCpuRequest decoded = DataloggerGetIdleCpuRequest_init_zero;
      if (pb_decode(&stream, DataloggerGetIdleCpuRequest_fields, &decoded))
        protocol_rx_DataloggerGetIdleCpuRequest(&decoded);
      break;
    }
    case MSG_ID_DataloggerGetIdleCpuResponse: {
      DataloggerGetIdleCpuResponse decoded = DataloggerGetIdleCpuResponse_init_zero;
      if (pb_decode(&stream, DataloggerGetIdleCpuResponse_fields, &decoded))
        protocol_rx_DataloggerGetIdleCpuResponse(&decoded);
      break;
    }
    default:
      break;
  }
}
