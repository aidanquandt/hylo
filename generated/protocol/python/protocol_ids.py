# Auto-generated from protocol.proto by tools/protocol_codegen/codegen_protocol.py

MSG_ID_AckResponse = 0
MSG_ID_SystemGetUuidRequest = 1
MSG_ID_SystemGetUuidResponse = 2
MSG_ID_SystemGetInfoRequest = 3
MSG_ID_SystemGetInfoResponse = 4
MSG_ID_SetTransportModeRequest = 5
MSG_ID_GetTransportModeRequest = 6
MSG_ID_GetTransportModeResponse = 7
MSG_ID_PingRequest = 8
MSG_ID_PingResponse = 9
MSG_ID_SetAddressRequest = 10
MSG_ID_SetAddressResponse = 11
MSG_ID_GetConfigRequest = 12
MSG_ID_GetConfigResponse = 13
MSG_ID_ImuGetStatusRequest = 14
MSG_ID_ImuGetStatusResponse = 15
MSG_ID_ImuGetDataRequest = 16
MSG_ID_ImuGetDataResponse = 17
MSG_ID_ImuStreamStartRequest = 18
MSG_ID_ImuStreamStopRequest = 19
MSG_ID_ImuStreamPayload = 20
MSG_ID_UwbNodeSetTypeRequest = 21
MSG_ID_UwbNodeGetTypeRequest = 22
MSG_ID_UwbNodeGetTypeResponse = 23
MSG_ID_UwbNodeSetPositionRequest = 24
MSG_ID_UwbNodeGetPositionRequest = 25
MSG_ID_UwbNodeGetPositionResponse = 26
MSG_ID_UwbNodeGetStatusRequest = 27
MSG_ID_UwbNodeGetStatusResponse = 28
MSG_ID_UwbGetStatusRequest = 29
MSG_ID_UwbGetStatusResponse = 30
MSG_ID_UwbGetStatsRequest = 31
MSG_ID_UwbGetStatsResponse = 32
MSG_ID_UwbStartRequest = 33
MSG_ID_UwbStopRequest = 34
MSG_ID_UwbResetStatsRequest = 35
MSG_ID_ErrorClearRequest = 36
MSG_ID_DataloggerGetTasksRequest = 37
MSG_ID_DataloggerGetTasksResponse = 38
MSG_ID_DataloggerGetStatsRequest = 39
MSG_ID_DataloggerGetStatsResponse = 40
MSG_ID_TwrGetStatusRequest = 41
MSG_ID_TwrGetStatusResponse = 42
MSG_ID_TwrGetResultRequest = 43
MSG_ID_TwrGetResultResponse = 44
MSG_ID_TwrRangeRequest = 45
MSG_ID_TwrMgrStartRequest = 46
MSG_ID_TwrMgrStopRequest = 47
MSG_ID_TwrMgrGetStatusRequest = 48
MSG_ID_TwrMgrGetStatusResponse = 49
MSG_ID_TwrMgrAddTargetRequest = 50
MSG_ID_TwrMgrRemoveTargetRequest = 51
MSG_ID_TwrMgrSetTargetsRequest = 52
MSG_ID_TwrMgrClearTargetsRequest = 53
MSG_ID_TwrMgrSetRangingRateRequest = 54
MSG_ID_TwrMgrGetRangingRateRequest = 55
MSG_ID_TwrMgrGetRangingRateResponse = 56
MSG_ID_StopwatchGetRequest = 57
MSG_ID_StopwatchGetResponse = 58
MSG_ID_StopwatchStartRequest = 59
MSG_ID_StopwatchStopRequest = 60
MSG_ID_OtaConfigSendAddressRequest = 61
MSG_ID_OtaConfigSendPositionRequest = 62
MSG_ID_OtaConfigSendTypeRequest = 63
MSG_ID_OtaConfigSendGpioRequest = 64
MSG_ID_OtaConfigSetTokenRequest = 65
MSG_ID_OtaConfigGetTokenRequest = 66
MSG_ID_OtaConfigGetTokenResponse = 67
MSG_ID_OtaConfigGetStatsRequest = 68
MSG_ID_OtaConfigGetStatsResponse = 69
MSG_ID_SensorFusionGetDebugRequest = 70
MSG_ID_SensorFusionSetDebugRequest = 71
MSG_ID_SensorFusionGetStatusRequest = 72
MSG_ID_SensorFusionGetStatusResponse = 73
MSG_ID_SensorFusionSetActiveRequest = 74
MSG_ID_SensorFusionGetImuEnabledRequest = 75
MSG_ID_SensorFusionGetImuEnabledResponse = 76
MSG_ID_SensorFusionSetImuEnabledRequest = 77
MSG_ID_SensorFusionSetNoiseRequest = 78
MSG_ID_SensorFusionGetNoiseRequest = 79
MSG_ID_SensorFusionGetNoiseResponse = 80
MSG_ID_SensorFusionGetConfigRequest = 81
MSG_ID_SensorFusionSetConfigRequest = 82
MSG_ID_BeaconPingRequest = 83
MSG_ID_BeaconPingResponse = 84
MSG_ID_ImuInitReturnedNullEvent = 85
MSG_ID_ImuProbeInitFailedEvent = 86
MSG_ID_ImuInvalidChipIdEvent = 87
MSG_ID_ImuAccelConfigFailedEvent = 88
MSG_ID_ImuGyroConfigFailedEvent = 89
MSG_ID_ImuFaultEvent = 90
MSG_ID_ImuFailedToPushEventToSensorFusionEvent = 91
MSG_ID_OtaConfigUnknownMessageTypeEvent = 92
MSG_ID_OtaConfigInvalidAddressEvent = 93
MSG_ID_OtaConfigAddressChangedEvent = 94
MSG_ID_OtaConfigPositionSetEvent = 95
MSG_ID_OtaConfigInvalidNodeTypeEvent = 96
MSG_ID_OtaConfigNodeTypeSetEvent = 97
MSG_ID_OtaConfigInvalidGpioPinEvent = 98
MSG_ID_OtaConfigInvalidGpioStateEvent = 99
MSG_ID_OtaConfigGpioSetEvent = 100
MSG_ID_OtaConfigResponseEvent = 101
MSG_ID_OtaConfigAuthFailedEvent = 102
MSG_ID_OtaConfigAckFromEvent = 103
MSG_ID_OtaConfigNoAckFromEvent = 104
MSG_ID_OtaConfigAuthTokenSetEvent = 105
MSG_ID_UwbFrameTooSmallEvent = 106
MSG_ID_UwbPayloadTooLargeEvent = 107
MSG_ID_UwbRetryAttemptEvent = 108
MSG_ID_UwbFaultManualStopRequiredEvent = 109
MSG_ID_UwbInvalidProtocolMessageEvent = 110
MSG_ID_UwbInvalidProtocolTypeEvent = 111
MSG_ID_UwbTxQueueFullEvent = 112
MSG_ID_UwbProtocolHandlerTableFullEvent = 113
MSG_ID_UwbNodeInvalidTypeEvent = 114
MSG_ID_UwbNodeTypeSetEvent = 115
MSG_ID_UwbNodePositionSetEvent = 116
MSG_ID_TwrFailedToRegisterProtocolHandlerEvent = 117
MSG_ID_TwrResponderAutoStartFailedEvent = 118
MSG_ID_TwrModuleNotInitializedEvent = 119
MSG_ID_ResponderFailedToSendMessageEvent = 120
MSG_ID_ResponderPollTooShortEvent = 121
MSG_ID_ResponderFinalTooShortEvent = 122
MSG_ID_ResponderFailedToSendFinalAckEvent = 123
MSG_ID_ResponderUnexpectedMessageTypeEvent = 124
MSG_ID_ResponderTimeoutWaitingForMessageEvent = 125
MSG_ID_ResponderFaultEvent = 126
MSG_ID_ResponderCompletedRangingEvent = 127
MSG_ID_InitiatorFailedToSendMessageEvent = 128
MSG_ID_InitiatorResponseTooShortEvent = 129
MSG_ID_InitiatorFinalAckTooShortEvent = 130
MSG_ID_InitiatorUnexpectedMessageTypeEvent = 131
MSG_ID_InitiatorFaultEvent = 132
MSG_ID_InitiatorDistanceCalculationFailedEvent = 133
MSG_ID_InitiatorUwbNotReadyEvent = 134
MSG_ID_TwrSmTxCompletionTimeoutEvent = 135
MSG_ID_TwrSmTxCompleteWrongStateEvent = 136
MSG_ID_TwrSmTxCompleteMsgIdMismatchEvent = 137
MSG_ID_TwrSmRxMessageWrongStateEvent = 138
MSG_ID_TwrSmTimeoutUnexpectedStateEvent = 139
MSG_ID_TwrSchedulerStrategyNotImplementedEvent = 140
MSG_ID_TwrSchedulerTargetListFullEvent = 141
MSG_ID_TwrSchedulerTargetAlreadyExistsEvent = 142
MSG_ID_TwrSchedulerAddedTargetEvent = 143
MSG_ID_TwrSchedulerRemovedTargetEvent = 144
MSG_ID_TwrSchedulerNullAddressesPointerEvent = 145
MSG_ID_TwrSchedulerTargetCountExceedsMaximumEvent = 146
MSG_ID_TwrSchedulerSetTargetsEvent = 147
MSG_ID_TwrSchedulerClearedAllTargetsEvent = 148
MSG_ID_TwrSchedulerTargetEnabledDisabledEvent = 149
MSG_ID_TwrSchedulerTargetBackingOffEvent = 150
MSG_ID_TwrSchedulerStrategySetEvent = 151
MSG_ID_TwrMgrFailedToPushRangingEventEvent = 152
MSG_ID_TwrMgrRangeEvent = 153
MSG_ID_TwrMgrEnteringIdleStateEvent = 154
MSG_ID_TwrMgrEnteringRangingStateEvent = 155
MSG_ID_TwrMgrEnteringFaultedStateEvent = 156
MSG_ID_TwrMgrCannotStartNoTargetsEvent = 157
MSG_ID_TwrMgrFailedToStartInitiatorEvent = 158
MSG_ID_TwrMgrInvalidRangingRateEvent = 159
MSG_ID_TwrMgrRangingRateSetEvent = 160
MSG_ID_DataloggerTimingMissesEvent = 161
MSG_ID_DataloggerLowMemoryEvent = 162
MSG_ID_AppDeviceNotInMappingTableEvent = 163
MSG_ID_AppUsingDefaultAddressEvent = 164
MSG_ID_AppFailedToInitDeviceIdEvent = 165
MSG_ID_WatchdogTaskFailureEvent = 166
MSG_ID_SystemFatalEvent = 167
MSG_ID_COUNT = 168

# Name list for dispatch
MSG_NAMES = (
    "AckResponse",
    "SystemGetUuidRequest",
    "SystemGetUuidResponse",
    "SystemGetInfoRequest",
    "SystemGetInfoResponse",
    "SetTransportModeRequest",
    "GetTransportModeRequest",
    "GetTransportModeResponse",
    "PingRequest",
    "PingResponse",
    "SetAddressRequest",
    "SetAddressResponse",
    "GetConfigRequest",
    "GetConfigResponse",
    "ImuGetStatusRequest",
    "ImuGetStatusResponse",
    "ImuGetDataRequest",
    "ImuGetDataResponse",
    "ImuStreamStartRequest",
    "ImuStreamStopRequest",
    "ImuStreamPayload",
    "UwbNodeSetTypeRequest",
    "UwbNodeGetTypeRequest",
    "UwbNodeGetTypeResponse",
    "UwbNodeSetPositionRequest",
    "UwbNodeGetPositionRequest",
    "UwbNodeGetPositionResponse",
    "UwbNodeGetStatusRequest",
    "UwbNodeGetStatusResponse",
    "UwbGetStatusRequest",
    "UwbGetStatusResponse",
    "UwbGetStatsRequest",
    "UwbGetStatsResponse",
    "UwbStartRequest",
    "UwbStopRequest",
    "UwbResetStatsRequest",
    "ErrorClearRequest",
    "DataloggerGetTasksRequest",
    "DataloggerGetTasksResponse",
    "DataloggerGetStatsRequest",
    "DataloggerGetStatsResponse",
    "TwrGetStatusRequest",
    "TwrGetStatusResponse",
    "TwrGetResultRequest",
    "TwrGetResultResponse",
    "TwrRangeRequest",
    "TwrMgrStartRequest",
    "TwrMgrStopRequest",
    "TwrMgrGetStatusRequest",
    "TwrMgrGetStatusResponse",
    "TwrMgrAddTargetRequest",
    "TwrMgrRemoveTargetRequest",
    "TwrMgrSetTargetsRequest",
    "TwrMgrClearTargetsRequest",
    "TwrMgrSetRangingRateRequest",
    "TwrMgrGetRangingRateRequest",
    "TwrMgrGetRangingRateResponse",
    "StopwatchGetRequest",
    "StopwatchGetResponse",
    "StopwatchStartRequest",
    "StopwatchStopRequest",
    "OtaConfigSendAddressRequest",
    "OtaConfigSendPositionRequest",
    "OtaConfigSendTypeRequest",
    "OtaConfigSendGpioRequest",
    "OtaConfigSetTokenRequest",
    "OtaConfigGetTokenRequest",
    "OtaConfigGetTokenResponse",
    "OtaConfigGetStatsRequest",
    "OtaConfigGetStatsResponse",
    "SensorFusionGetDebugRequest",
    "SensorFusionSetDebugRequest",
    "SensorFusionGetStatusRequest",
    "SensorFusionGetStatusResponse",
    "SensorFusionSetActiveRequest",
    "SensorFusionGetImuEnabledRequest",
    "SensorFusionGetImuEnabledResponse",
    "SensorFusionSetImuEnabledRequest",
    "SensorFusionSetNoiseRequest",
    "SensorFusionGetNoiseRequest",
    "SensorFusionGetNoiseResponse",
    "SensorFusionGetConfigRequest",
    "SensorFusionSetConfigRequest",
    "BeaconPingRequest",
    "BeaconPingResponse",
    "ImuInitReturnedNullEvent",
    "ImuProbeInitFailedEvent",
    "ImuInvalidChipIdEvent",
    "ImuAccelConfigFailedEvent",
    "ImuGyroConfigFailedEvent",
    "ImuFaultEvent",
    "ImuFailedToPushEventToSensorFusionEvent",
    "OtaConfigUnknownMessageTypeEvent",
    "OtaConfigInvalidAddressEvent",
    "OtaConfigAddressChangedEvent",
    "OtaConfigPositionSetEvent",
    "OtaConfigInvalidNodeTypeEvent",
    "OtaConfigNodeTypeSetEvent",
    "OtaConfigInvalidGpioPinEvent",
    "OtaConfigInvalidGpioStateEvent",
    "OtaConfigGpioSetEvent",
    "OtaConfigResponseEvent",
    "OtaConfigAuthFailedEvent",
    "OtaConfigAckFromEvent",
    "OtaConfigNoAckFromEvent",
    "OtaConfigAuthTokenSetEvent",
    "UwbFrameTooSmallEvent",
    "UwbPayloadTooLargeEvent",
    "UwbRetryAttemptEvent",
    "UwbFaultManualStopRequiredEvent",
    "UwbInvalidProtocolMessageEvent",
    "UwbInvalidProtocolTypeEvent",
    "UwbTxQueueFullEvent",
    "UwbProtocolHandlerTableFullEvent",
    "UwbNodeInvalidTypeEvent",
    "UwbNodeTypeSetEvent",
    "UwbNodePositionSetEvent",
    "TwrFailedToRegisterProtocolHandlerEvent",
    "TwrResponderAutoStartFailedEvent",
    "TwrModuleNotInitializedEvent",
    "ResponderFailedToSendMessageEvent",
    "ResponderPollTooShortEvent",
    "ResponderFinalTooShortEvent",
    "ResponderFailedToSendFinalAckEvent",
    "ResponderUnexpectedMessageTypeEvent",
    "ResponderTimeoutWaitingForMessageEvent",
    "ResponderFaultEvent",
    "ResponderCompletedRangingEvent",
    "InitiatorFailedToSendMessageEvent",
    "InitiatorResponseTooShortEvent",
    "InitiatorFinalAckTooShortEvent",
    "InitiatorUnexpectedMessageTypeEvent",
    "InitiatorFaultEvent",
    "InitiatorDistanceCalculationFailedEvent",
    "InitiatorUwbNotReadyEvent",
    "TwrSmTxCompletionTimeoutEvent",
    "TwrSmTxCompleteWrongStateEvent",
    "TwrSmTxCompleteMsgIdMismatchEvent",
    "TwrSmRxMessageWrongStateEvent",
    "TwrSmTimeoutUnexpectedStateEvent",
    "TwrSchedulerStrategyNotImplementedEvent",
    "TwrSchedulerTargetListFullEvent",
    "TwrSchedulerTargetAlreadyExistsEvent",
    "TwrSchedulerAddedTargetEvent",
    "TwrSchedulerRemovedTargetEvent",
    "TwrSchedulerNullAddressesPointerEvent",
    "TwrSchedulerTargetCountExceedsMaximumEvent",
    "TwrSchedulerSetTargetsEvent",
    "TwrSchedulerClearedAllTargetsEvent",
    "TwrSchedulerTargetEnabledDisabledEvent",
    "TwrSchedulerTargetBackingOffEvent",
    "TwrSchedulerStrategySetEvent",
    "TwrMgrFailedToPushRangingEventEvent",
    "TwrMgrRangeEvent",
    "TwrMgrEnteringIdleStateEvent",
    "TwrMgrEnteringRangingStateEvent",
    "TwrMgrEnteringFaultedStateEvent",
    "TwrMgrCannotStartNoTargetsEvent",
    "TwrMgrFailedToStartInitiatorEvent",
    "TwrMgrInvalidRangingRateEvent",
    "TwrMgrRangingRateSetEvent",
    "DataloggerTimingMissesEvent",
    "DataloggerLowMemoryEvent",
    "AppDeviceNotInMappingTableEvent",
    "AppUsingDefaultAddressEvent",
    "AppFailedToInitDeviceIdEvent",
    "WatchdogTaskFailureEvent",
    "SystemFatalEvent",
)
