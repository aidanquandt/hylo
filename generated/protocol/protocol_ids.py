# Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py

MSG_ID_AckResponse = 0
MSG_ID_SystemGetUuidRequest = 1
MSG_ID_SystemGetUuidResponse = 2
MSG_ID_SystemGetInfoRequest = 3
MSG_ID_SystemGetInfoResponse = 4
MSG_ID_PingRequest = 5
MSG_ID_PingResponse = 6
MSG_ID_SetAddressRequest = 7
MSG_ID_SetAddressResponse = 8
MSG_ID_GetConfigRequest = 9
MSG_ID_GetConfigResponse = 10
MSG_ID_ImuGetStatusRequest = 11
MSG_ID_ImuGetStatusResponse = 12
MSG_ID_ImuGetDataRequest = 13
MSG_ID_ImuGetDataResponse = 14
MSG_ID_ImuStreamStartRequest = 15
MSG_ID_ImuStreamStopRequest = 16
MSG_ID_ImuStreamPayload = 17
MSG_ID_UwbNodeSetTypeRequest = 18
MSG_ID_UwbNodeGetTypeRequest = 19
MSG_ID_UwbNodeGetTypeResponse = 20
MSG_ID_UwbNodeSetPositionRequest = 21
MSG_ID_UwbNodeGetPositionRequest = 22
MSG_ID_UwbNodeGetPositionResponse = 23
MSG_ID_UwbNodeGetStatusRequest = 24
MSG_ID_UwbNodeGetStatusResponse = 25
MSG_ID_UwbGetStatusRequest = 26
MSG_ID_UwbGetStatusResponse = 27
MSG_ID_UwbGetStatsRequest = 28
MSG_ID_UwbGetStatsResponse = 29
MSG_ID_UwbStartRequest = 30
MSG_ID_UwbStopRequest = 31
MSG_ID_UwbResetStatsRequest = 32
MSG_ID_ErrorClearRequest = 33
MSG_ID_DataloggerGetTasksRequest = 34
MSG_ID_DataloggerGetTasksResponse = 35
MSG_ID_DataloggerGetStatsRequest = 36
MSG_ID_DataloggerGetStatsResponse = 37
MSG_ID_TwrGetStatusRequest = 38
MSG_ID_TwrGetStatusResponse = 39
MSG_ID_TwrGetResultRequest = 40
MSG_ID_TwrGetResultResponse = 41
MSG_ID_TwrRangeRequest = 42
MSG_ID_TwrMgrStartRequest = 43
MSG_ID_TwrMgrStopRequest = 44
MSG_ID_TwrMgrGetStatusRequest = 45
MSG_ID_TwrMgrGetStatusResponse = 46
MSG_ID_TwrMgrAddTargetRequest = 47
MSG_ID_TwrMgrRemoveTargetRequest = 48
MSG_ID_TwrMgrSetTargetsRequest = 49
MSG_ID_TwrMgrClearTargetsRequest = 50
MSG_ID_TwrMgrSetRangingRateRequest = 51
MSG_ID_TwrMgrGetRangingRateRequest = 52
MSG_ID_TwrMgrGetRangingRateResponse = 53
MSG_ID_StopwatchGetRequest = 54
MSG_ID_StopwatchGetResponse = 55
MSG_ID_StopwatchStartRequest = 56
MSG_ID_StopwatchStopRequest = 57
MSG_ID_OtaConfigSendAddressRequest = 58
MSG_ID_OtaConfigSendPositionRequest = 59
MSG_ID_OtaConfigSendTypeRequest = 60
MSG_ID_OtaConfigSendGpioRequest = 61
MSG_ID_OtaConfigSetTokenRequest = 62
MSG_ID_OtaConfigGetTokenRequest = 63
MSG_ID_OtaConfigGetTokenResponse = 64
MSG_ID_OtaConfigGetStatsRequest = 65
MSG_ID_OtaConfigGetStatsResponse = 66
MSG_ID_SensorFusionGetDebugRequest = 67
MSG_ID_SensorFusionSetDebugRequest = 68
MSG_ID_SensorFusionGetStatusRequest = 69
MSG_ID_SensorFusionGetStatusResponse = 70
MSG_ID_SensorFusionSetActiveRequest = 71
MSG_ID_SensorFusionGetImuEnabledRequest = 72
MSG_ID_SensorFusionGetImuEnabledResponse = 73
MSG_ID_SensorFusionSetImuEnabledRequest = 74
MSG_ID_SensorFusionSetNoiseRequest = 75
MSG_ID_SensorFusionGetNoiseRequest = 76
MSG_ID_SensorFusionGetNoiseResponse = 77
MSG_ID_SensorFusionGetConfigRequest = 78
MSG_ID_SensorFusionSetConfigRequest = 79
MSG_ID_BeaconPingRequest = 80
MSG_ID_BeaconPingResponse = 81
MSG_ID_ImuInitReturnedNullEvent = 82
MSG_ID_ImuProbeInitFailedEvent = 83
MSG_ID_ImuInvalidChipIdEvent = 84
MSG_ID_ImuAccelConfigFailedEvent = 85
MSG_ID_ImuGyroConfigFailedEvent = 86
MSG_ID_ImuFaultEvent = 87
MSG_ID_ImuFailedToPushEventToSensorFusionEvent = 88
MSG_ID_OtaConfigUnknownMessageTypeEvent = 89
MSG_ID_OtaConfigInvalidAddressEvent = 90
MSG_ID_OtaConfigAddressChangedEvent = 91
MSG_ID_OtaConfigPositionSetEvent = 92
MSG_ID_OtaConfigInvalidNodeTypeEvent = 93
MSG_ID_OtaConfigNodeTypeSetEvent = 94
MSG_ID_OtaConfigInvalidGpioPinEvent = 95
MSG_ID_OtaConfigInvalidGpioStateEvent = 96
MSG_ID_OtaConfigGpioSetEvent = 97
MSG_ID_OtaConfigResponseEvent = 98
MSG_ID_OtaConfigAuthFailedEvent = 99
MSG_ID_OtaConfigAckFromEvent = 100
MSG_ID_OtaConfigNoAckFromEvent = 101
MSG_ID_OtaConfigAuthTokenSetEvent = 102
MSG_ID_UwbFrameTooSmallEvent = 103
MSG_ID_UwbPayloadTooLargeEvent = 104
MSG_ID_UwbRetryAttemptEvent = 105
MSG_ID_UwbFaultManualStopRequiredEvent = 106
MSG_ID_UwbInvalidProtocolMessageEvent = 107
MSG_ID_UwbInvalidProtocolTypeEvent = 108
MSG_ID_UwbTxQueueFullEvent = 109
MSG_ID_UwbProtocolHandlerTableFullEvent = 110
MSG_ID_UwbNodeInvalidTypeEvent = 111
MSG_ID_UwbNodeTypeSetEvent = 112
MSG_ID_UwbNodePositionSetEvent = 113
MSG_ID_TwrFailedToRegisterProtocolHandlerEvent = 114
MSG_ID_TwrResponderAutoStartFailedEvent = 115
MSG_ID_TwrModuleNotInitializedEvent = 116
MSG_ID_ResponderFailedToSendMessageEvent = 117
MSG_ID_ResponderPollTooShortEvent = 118
MSG_ID_ResponderFinalTooShortEvent = 119
MSG_ID_ResponderFailedToSendFinalAckEvent = 120
MSG_ID_ResponderUnexpectedMessageTypeEvent = 121
MSG_ID_ResponderTimeoutWaitingForMessageEvent = 122
MSG_ID_ResponderFaultEvent = 123
MSG_ID_ResponderCompletedRangingEvent = 124
MSG_ID_InitiatorFailedToSendMessageEvent = 125
MSG_ID_InitiatorResponseTooShortEvent = 126
MSG_ID_InitiatorFinalAckTooShortEvent = 127
MSG_ID_InitiatorUnexpectedMessageTypeEvent = 128
MSG_ID_InitiatorFaultEvent = 129
MSG_ID_InitiatorDistanceCalculationFailedEvent = 130
MSG_ID_InitiatorUwbNotReadyEvent = 131
MSG_ID_TwrSmTxCompletionTimeoutEvent = 132
MSG_ID_TwrSmTxCompleteWrongStateEvent = 133
MSG_ID_TwrSmTxCompleteMsgIdMismatchEvent = 134
MSG_ID_TwrSmRxMessageWrongStateEvent = 135
MSG_ID_TwrSmTimeoutUnexpectedStateEvent = 136
MSG_ID_TwrSchedulerStrategyNotImplementedEvent = 137
MSG_ID_TwrSchedulerTargetListFullEvent = 138
MSG_ID_TwrSchedulerTargetAlreadyExistsEvent = 139
MSG_ID_TwrSchedulerAddedTargetEvent = 140
MSG_ID_TwrSchedulerRemovedTargetEvent = 141
MSG_ID_TwrSchedulerNullAddressesPointerEvent = 142
MSG_ID_TwrSchedulerTargetCountExceedsMaximumEvent = 143
MSG_ID_TwrSchedulerSetTargetsEvent = 144
MSG_ID_TwrSchedulerClearedAllTargetsEvent = 145
MSG_ID_TwrSchedulerTargetEnabledDisabledEvent = 146
MSG_ID_TwrSchedulerTargetBackingOffEvent = 147
MSG_ID_TwrSchedulerStrategySetEvent = 148
MSG_ID_TwrMgrFailedToPushRangingEventEvent = 149
MSG_ID_TwrMgrRangeEvent = 150
MSG_ID_TwrMgrEnteringIdleStateEvent = 151
MSG_ID_TwrMgrEnteringRangingStateEvent = 152
MSG_ID_TwrMgrEnteringFaultedStateEvent = 153
MSG_ID_TwrMgrCannotStartNoTargetsEvent = 154
MSG_ID_TwrMgrFailedToStartInitiatorEvent = 155
MSG_ID_TwrMgrInvalidRangingRateEvent = 156
MSG_ID_TwrMgrRangingRateSetEvent = 157
MSG_ID_DataloggerTimingMissesEvent = 158
MSG_ID_DataloggerLowMemoryEvent = 159
MSG_ID_AppDeviceNotInMappingTableEvent = 160
MSG_ID_AppUsingDefaultAddressEvent = 161
MSG_ID_AppFailedToInitDeviceIdEvent = 162
MSG_ID_WatchdogTaskFailureEvent = 163
MSG_ID_SystemFatalEvent = 164
MSG_ID_COUNT = 165

# Name list for dispatch
MSG_NAMES = (
    "AckResponse",
    "SystemGetUuidRequest",
    "SystemGetUuidResponse",
    "SystemGetInfoRequest",
    "SystemGetInfoResponse",
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
