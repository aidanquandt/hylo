/*
 * UART protocol RX handlers: implement protocol_rx_* for every Request.
 * Strong implementations override weak stubs in protocol_dispatch.c.
 * Fire-and-forget requests respond with AckResponse.
 */
#include "protocol_dispatch.h"
#include "protocol_tx.h"
#include "protocol.pb.h"
#include "uart_framing.h"
#include "common.h"
#include "datalogger.h"
#include "imu.h"
#include "imu_driver.h"
#include "initiator.h"
#include "ota_config.h"
#include "responder.h"
#include "sensor_fusion.h"
#include "kalman_core.h"
#include "stopwatch.h"
#include "system_driver.h"
#include "twr.h"
#include "twr_manager.h"
#include "twr_scheduler.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Private state
 *---------------------------------------------------------------------------*/
static uint32_t s_ping_seq;

/* IMU stream: 10 Hz task sends ImuStreamPayload; mode 0 = array (per-IMU), 1 = avg */
#define IMU_STREAM_TASK_STACK  256
#define IMU_STREAM_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define IMU_STREAM_PERIOD_MS    100

static volatile bool s_imu_stream_running;
static volatile uint32_t s_imu_stream_mode; /* 0 = array, 1 = avg */
static TaskHandle_t s_imu_stream_task_handle;

static void imu_stream_task_fn(void *arg);
static void imu_stream_send_payload(uint32_t imu_index, float accel_x, float accel_y, float accel_z,
                                    float gyro_x, float gyro_y, float gyro_z, float temp_c);

/*---------------------------------------------------------------------------
 * Helpers: NodeType <-> uwb_node_type_e (same numeric values)
 *---------------------------------------------------------------------------*/
static NodeType node_type_from_uwb(uwb_node_type_e t)
{
    return (NodeType)((uint32_t)t);
}

static uwb_node_type_e uwb_node_type_from_proto(NodeType t)
{
    if ((uint32_t)t > (uint32_t)UWB_NODE_TYPE_HYBRID)
        return UWB_NODE_TYPE_TAG;
    return (uwb_node_type_e)(uint32_t)t;
}

/*---------------------------------------------------------------------------
 * System
 *---------------------------------------------------------------------------*/
void protocol_rx_SystemGetUuidRequest(const SystemGetUuidRequest *msg)
{
    (void)msg;
    SystemGetUuidResponse r = SystemGetUuidResponse_init_zero;
    r.uuid_word0 = system_driver_get_uuid_word(0);
    r.uuid_word1 = system_driver_get_uuid_word(1);
    r.uuid_word2 = system_driver_get_uuid_word(2);
    protocol_tx_SystemGetUuidResponse(&r);
}

void protocol_rx_SystemGetInfoRequest(const SystemGetInfoRequest *msg)
{
    (void)msg;
    SystemGetInfoResponse r = SystemGetInfoResponse_init_zero;
    system_driver_device_info_t info;
    if (system_driver_device_get_info(&info))
    {
        const char *name = system_driver_device_get_name();
        if (name != NULL)
        {
            strncpy(r.info, name, sizeof(r.info) - 1);
            r.info[sizeof(r.info) - 1] = '\0';
        }
    }
    protocol_tx_SystemGetInfoResponse(&r);
}

/*---------------------------------------------------------------------------
 * Beacon (UART ping)
 *---------------------------------------------------------------------------*/
void protocol_rx_PingRequest(const PingRequest *msg)
{
    (void)msg;
    PingResponse r = PingResponse_init_zero;
    r.seq = s_ping_seq++;
    protocol_tx_PingResponse(&r);
}

/*---------------------------------------------------------------------------
 * UWB node config (address, PAN)
 *---------------------------------------------------------------------------*/
void protocol_rx_GetConfigRequest(const GetConfigRequest *msg)
{
    (void)msg;
    GetConfigResponse r = GetConfigResponse_init_zero;
    r.pan_id   = uwb_get_pan_id();
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

/*---------------------------------------------------------------------------
 * IMU
 *---------------------------------------------------------------------------*/
void protocol_rx_ImuGetStatusRequest(const ImuGetStatusRequest *msg)
{
    (void)msg;
    imu_status_t st;
    imu_get_status(&st);
    ImuGetStatusResponse r = ImuGetStatusResponse_init_zero;
    r.active     = (st.state == IMU_STATE_ACTIVE);
    r.fault_code = st.fault_code;
    r.state      = (uint32_t)st.state;
    protocol_tx_ImuGetStatusResponse(&r);
}

void protocol_rx_ImuGetDataRequest(const ImuGetDataRequest *msg)
{
    (void)msg;
    ImuGetDataResponse r = ImuGetDataResponse_init_zero;
    imu_data_t data;
    if (imu_get_data(&data))
    {
        r.accel_x = data.accel.x;
        r.accel_y = data.accel.y;
        r.accel_z = data.accel.z;
        r.gyro_x  = data.gyro.x;
        r.gyro_y  = data.gyro.y;
        r.gyro_z  = data.gyro.z;
        r.temp_c  = data.temperature;
    }
    protocol_tx_ImuGetDataResponse(&r);
}

static void imu_stream_send_payload(uint32_t imu_index, float accel_x, float accel_y, float accel_z,
                                    float gyro_x, float gyro_y, float gyro_z, float temp_c)
{
    ImuStreamPayload p = ImuStreamPayload_init_zero;
    p.accel_x  = accel_x;
    p.accel_y  = accel_y;
    p.accel_z  = accel_z;
    p.gyro_x   = gyro_x;
    p.gyro_y   = gyro_y;
    p.gyro_z   = gyro_z;
    p.temp_c   = temp_c;
    p.imu_index = imu_index;
    protocol_tx_ImuStreamPayload(&p);
}

static void imu_stream_task_fn(void *arg)
{
    (void)arg;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(IMU_STREAM_PERIOD_MS));
        if (!s_imu_stream_running)
            continue;

        if (s_imu_stream_mode == 1)
        {
            /* Avg: single payload with aggregate data */
            imu_data_t data;
            if (imu_get_data(&data))
            {
                imu_stream_send_payload(0,
                                        data.accel.x, data.accel.y, data.accel.z,
                                        data.gyro.x, data.gyro.y, data.gyro.z,
                                        data.temperature);
            }
        }
        else
        {
            /* Array: one payload per active IMU */
            for (uint8_t i = 0; i < imu_get_device_count(); i++)
            {
                imu_data_t data;
                if (imu_get_individual_data((imu_device_e)i, &data))
                {
                    imu_stream_send_payload((uint32_t)i,
                                            data.accel.x, data.accel.y, data.accel.z,
                                            data.gyro.x, data.gyro.y, data.gyro.z,
                                            data.temperature);
                }
            }
        }
    }
}

void protocol_rx_ImuStreamStartRequest(const ImuStreamStartRequest *msg)
{
    s_imu_stream_mode   = (msg->mode != 0) ? 1 : 0;
    s_imu_stream_running = true;
    if (s_imu_stream_task_handle == NULL)
    {
        BaseType_t ok = xTaskCreate(imu_stream_task_fn, "imu_str", IMU_STREAM_TASK_STACK, NULL,
                                    IMU_STREAM_TASK_PRIORITY, &s_imu_stream_task_handle);
        (void)ok;
    }
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_ImuStreamStopRequest(const ImuStreamStopRequest *msg)
{
    (void)msg;
    s_imu_stream_running = false;
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * UWB node (type, position, status)
 *---------------------------------------------------------------------------*/
void protocol_rx_UwbNodeSetTypeRequest(const UwbNodeSetTypeRequest *msg)
{
    uwb_node_set_type(uwb_node_type_from_proto(msg->node_type));
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_UwbNodeGetTypeRequest(const UwbNodeGetTypeRequest *msg)
{
    (void)msg;
    UwbNodeGetTypeResponse r = UwbNodeGetTypeResponse_init_zero;
    r.node_type = node_type_from_uwb(uwb_node_get_type());
    protocol_tx_UwbNodeGetTypeResponse(&r);
}

void protocol_rx_UwbNodeSetPositionRequest(const UwbNodeSetPositionRequest *msg)
{
    vec3_t pos = { .x = msg->x, .y = msg->y, .z = msg->z };
    uwb_node_set_position(&pos);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_UwbNodeGetPositionRequest(const UwbNodeGetPositionRequest *msg)
{
    (void)msg;
    UwbNodeGetPositionResponse r = UwbNodeGetPositionResponse_init_zero;
    vec3_t pos;
    if (uwb_node_get_position(&pos))
    {
        r.x = pos.x;
        r.y = pos.y;
        r.z = pos.z;
        r.position_known = true;
    }
    protocol_tx_UwbNodeGetPositionResponse(&r);
}

void protocol_rx_UwbNodeGetStatusRequest(const UwbNodeGetStatusRequest *msg)
{
    (void)msg;
    uwb_node_config_t cfg;
    uwb_node_get_status(&cfg);
    UwbNodeGetStatusResponse r = UwbNodeGetStatusResponse_init_zero;
    r.node_type     = node_type_from_uwb(cfg.type);
    r.pos_x         = cfg.position_x;
    r.pos_y         = cfg.position_y;
    r.pos_z         = cfg.position_z;
    r.position_known = cfg.position_known;
    r.address       = uwb_get_address();
    r.pan_id        = uwb_get_pan_id();
    protocol_tx_UwbNodeGetStatusResponse(&r);
}

/*---------------------------------------------------------------------------
 * UWB (radio start/stop/stats)
 *---------------------------------------------------------------------------*/
void protocol_rx_UwbGetStatusRequest(const UwbGetStatusRequest *msg)
{
    (void)msg;
    uwb_status_t st;
    uwb_get_status(&st);
    UwbGetStatusResponse r = UwbGetStatusResponse_init_zero;
    r.initialized = (st.state == UWB_STATE_ACTIVE);
    r.fault_code  = st.fault_code;
    protocol_tx_UwbGetStatusResponse(&r);
}

void protocol_rx_UwbGetStatsRequest(const UwbGetStatsRequest *msg)
{
    (void)msg;
    uwb_driver_stats_t drv;
    uwb_get_driver_statistics(&drv);
    UwbGetStatsResponse r = UwbGetStatsResponse_init_zero;
    r.tx_ok   = drv.tx_done_count;
    r.tx_fail = drv.TXF + uwb_get_tx_timeouts();
    r.rx_ok   = drv.rx_ok_count;
    protocol_tx_UwbGetStatsResponse(&r);
}

void protocol_rx_UwbStartRequest(const UwbStartRequest *msg)
{
    (void)msg;
    uwb_start();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_UwbStopRequest(const UwbStopRequest *msg)
{
    (void)msg;
    uwb_stop();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_UwbResetStatsRequest(const UwbResetStatsRequest *msg)
{
    (void)msg;
    uwb_reset_rx_stats();
    uwb_reset_protocol_stats();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * Error (clear; no history on device)
 *---------------------------------------------------------------------------*/
void protocol_rx_ErrorClearRequest(const ErrorClearRequest *msg)
{
    (void)msg;
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * Datalogger
 *---------------------------------------------------------------------------*/
void protocol_rx_DataloggerGetTasksRequest(const DataloggerGetTasksRequest *msg)
{
    (void)msg;
    task_cpu_info_t tasks[16];
    uint32_t n = datalogger_get_task_usage(tasks, 16);
    DataloggerGetTasksResponse r = DataloggerGetTasksResponse_init_zero;
    r.task_names_count = (pb_size_t)n;
    for (uint32_t i = 0; i < n && i < 16; i++)
    {
        if (tasks[i].task_name != NULL)
        {
            strncpy(r.task_names[i], tasks[i].task_name, 32);
            r.task_names[i][32] = '\0';
        }
    }
    protocol_tx_DataloggerGetTasksResponse(&r);
}

void protocol_rx_DataloggerGetStatsRequest(const DataloggerGetStatsRequest *msg)
{
    (void)msg;
    system_stats_t st;
    task_cpu_info_t buf[16];
    datalogger_get_system_stats(&st, buf, 16);
    DataloggerGetStatsResponse r = DataloggerGetStatsResponse_init_zero;
    r.task_count = st.num_tasks;
    r.free_heap  = st.current_free_heap;
    protocol_tx_DataloggerGetStatsResponse(&r);
}

/*---------------------------------------------------------------------------
 * TWR
 *---------------------------------------------------------------------------*/
void protocol_rx_TwrGetStatusRequest(const TwrGetStatusRequest *msg)
{
    (void)msg;
    TwrGetStatusResponse r = TwrGetStatusResponse_init_zero;
    initiator_status_t ini;
    responder_status_t resp;
    initiator_get_status(&ini);
    responder_get_status(&resp);
    r.state             = (uint32_t)ini.state;
    r.role              = (uint32_t)TWR_ROLE_INITIATOR; /* report initiator role for tag */
    r.initiator_running = initiator_is_ranging();
    r.responder_running = (resp.state != RESPONDER_STATE_IDLE);
    protocol_tx_TwrGetStatusResponse(&r);
}

void protocol_rx_TwrGetResultRequest(const TwrGetResultRequest *msg)
{
    (void)msg;
    twr_result_t res;
    bool valid = initiator_get_last_result(&res);
    TwrGetResultResponse r = TwrGetResultResponse_init_zero;
    r.distance_m    = res.distance_m;
    r.responder_addr = res.remote_addr;
    r.valid         = valid && res.valid;
    protocol_tx_TwrGetResultResponse(&r);
}

void protocol_rx_TwrRangeRequest(const TwrRangeRequest *msg)
{
    bool ok = twr_start_ranging((uint16_t)msg->target_addr);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * TWR manager
 *---------------------------------------------------------------------------*/
void protocol_rx_TwrMgrStartRequest(const TwrMgrStartRequest *msg)
{
    (void)msg;
    bool ok = twr_manager_start();
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrStopRequest(const TwrMgrStopRequest *msg)
{
    (void)msg;
    twr_manager_stop();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrGetStatusRequest(const TwrMgrGetStatusRequest *msg)
{
    (void)msg;
    twr_scheduler_status_t st;
    twr_scheduler_get_status(&st);
    TwrMgrGetStatusResponse r = TwrMgrGetStatusResponse_init_zero;
    r.state         = twr_manager_is_active() ? 1 : 0;
    r.target_count = st.target_count;
    r.ranging_rate_hz = twr_manager_get_ranging_rate();
    protocol_tx_TwrMgrGetStatusResponse(&r);
}

void protocol_rx_TwrMgrAddTargetRequest(const TwrMgrAddTargetRequest *msg)
{
    bool ok = twr_manager_add_target((uint16_t)msg->address);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrRemoveTargetRequest(const TwrMgrRemoveTargetRequest *msg)
{
    bool ok = twr_manager_remove_target((uint16_t)msg->address);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrSetTargetsRequest(const TwrMgrSetTargetsRequest *msg)
{
    uint16_t addrs[16];
    pb_size_t n = msg->addresses_count;
    if (n > 16)
        n = 16;
    for (pb_size_t i = 0; i < n; i++)
        addrs[i] = (uint16_t)msg->addresses[i];
    bool ok = twr_scheduler_set_targets(addrs, (uint8_t)n);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrClearTargetsRequest(const TwrMgrClearTargetsRequest *msg)
{
    (void)msg;
    twr_scheduler_clear_all();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrSetRangingRateRequest(const TwrMgrSetRangingRateRequest *msg)
{
    bool ok = twr_manager_set_ranging_rate((uint16_t)msg->rate_hz);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_TwrMgrGetRangingRateRequest(const TwrMgrGetRangingRateRequest *msg)
{
    (void)msg;
    TwrMgrGetRangingRateResponse r = TwrMgrGetRangingRateResponse_init_zero;
    r.rate_hz = twr_manager_get_ranging_rate();
    protocol_tx_TwrMgrGetRangingRateResponse(&r);
}

/*---------------------------------------------------------------------------
 * Stopwatch
 *---------------------------------------------------------------------------*/
void protocol_rx_StopwatchGetRequest(const StopwatchGetRequest *msg)
{
    (void)msg;
    StopwatchGetResponse r = StopwatchGetResponse_init_zero;
    r.elapsed_ms = stopwatch_elapsed_us(0) / 1000;
    r.running    = stopwatch_is_running(0);
    protocol_tx_StopwatchGetResponse(&r);
}

void protocol_rx_StopwatchStartRequest(const StopwatchStartRequest *msg)
{
    (void)msg;
    stopwatch_start(0);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_StopwatchStopRequest(const StopwatchStopRequest *msg)
{
    (void)msg;
    stopwatch_stop(0);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * OTA config
 *---------------------------------------------------------------------------*/
void protocol_rx_OtaConfigSendAddressRequest(const OtaConfigSendAddressRequest *msg)
{
    bool ok = ota_config_send_set_address((uint16_t)msg->target_addr, (uint16_t)msg->new_address,
                                          (uint16_t)msg->pan_id);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_OtaConfigSendPositionRequest(const OtaConfigSendPositionRequest *msg)
{
    vec3_t pos = { .x = msg->x, .y = msg->y, .z = msg->z };
    bool ok = ota_config_send_set_position((uint16_t)msg->target_addr, &pos);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_OtaConfigSendTypeRequest(const OtaConfigSendTypeRequest *msg)
{
    bool ok = ota_config_send_set_node_type((uint16_t)msg->target_addr,
                                            uwb_node_type_from_proto(msg->node_type));
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_OtaConfigSendGpioRequest(const OtaConfigSendGpioRequest *msg)
{
    bool ok = ota_config_send_set_gpio((uint16_t)msg->target_addr, (uint8_t)msg->pin,
                                       (uint8_t)msg->state);
    AckResponse ack = AckResponse_init_zero;
    ack.success = ok;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_OtaConfigSetTokenRequest(const OtaConfigSetTokenRequest *msg)
{
    ota_config_set_auth_token(msg->token);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_OtaConfigGetTokenRequest(const OtaConfigGetTokenRequest *msg)
{
    (void)msg;
    OtaConfigGetTokenResponse r = OtaConfigGetTokenResponse_init_zero;
    r.token = ota_config_get_auth_token();
    protocol_tx_OtaConfigGetTokenResponse(&r);
}

void protocol_rx_OtaConfigGetStatsRequest(const OtaConfigGetStatsRequest *msg)
{
    (void)msg;
    uint32_t sent = 0, recv = 0, auth_fail = 0;
    ota_config_get_stats(&sent, &recv, &auth_fail);
    OtaConfigGetStatsResponse r = OtaConfigGetStatsResponse_init_zero;
    r.requests_sent     = sent;
    r.responses_received = recv;
    protocol_tx_OtaConfigGetStatsResponse(&r);
}

/*---------------------------------------------------------------------------
 * Sensor fusion
 *---------------------------------------------------------------------------*/
void protocol_rx_SensorFusionGetDebugRequest(const SensorFusionGetDebugRequest *msg)
{
    (void)msg;
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionSetDebugRequest(const SensorFusionSetDebugRequest *msg)
{
    sensor_fusion_enable_debug_prints(msg->debug_flags != 0);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionGetStatusRequest(const SensorFusionGetStatusRequest *msg)
{
    (void)msg;
    SensorFusionGetStatusResponse r = SensorFusionGetStatusResponse_init_zero;
    r.active = sensor_fusion_is_active();
    sensor_fusion_position_t pos;
    if (sensor_fusion_get_position(&pos))
    {
        r.pos_x = pos.x;
        r.pos_y = pos.y;
        r.pos_z = pos.z;
        r.confidence = pos.confidence;
    }
    protocol_tx_SensorFusionGetStatusResponse(&r);
}

void protocol_rx_SensorFusionSetActiveRequest(const SensorFusionSetActiveRequest *msg)
{
    if (msg->active)
        sensor_fusion_start();
    else
        sensor_fusion_stop();
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionGetImuEnabledRequest(const SensorFusionGetImuEnabledRequest *msg)
{
    (void)msg;
    SensorFusionGetImuEnabledResponse r = SensorFusionGetImuEnabledResponse_init_zero;
    r.imu_enabled = sensor_fusion_get_imu_enabled();
    protocol_tx_SensorFusionGetImuEnabledResponse(&r);
}

void protocol_rx_SensorFusionSetImuEnabledRequest(const SensorFusionSetImuEnabledRequest *msg)
{
    sensor_fusion_enable_imu(msg->imu_enabled);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionSetNoiseRequest(const SensorFusionSetNoiseRequest *msg)
{
    sensor_fusion_set_process_noise(msg->pos, msg->vel, msg->att);
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionGetNoiseRequest(const SensorFusionGetNoiseRequest *msg)
{
    (void)msg;
    SensorFusionGetNoiseResponse r = SensorFusionGetNoiseResponse_init_zero;
    float pos, vel, att;
    sensor_fusion_get_process_noise(&pos, &vel, &att);
    r.pos = pos;
    r.vel = vel;
    r.att = att;
    protocol_tx_SensorFusionGetNoiseResponse(&r);
}

void protocol_rx_SensorFusionGetConfigRequest(const SensorFusionGetConfigRequest *msg)
{
    (void)msg;
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

void protocol_rx_SensorFusionSetConfigRequest(const SensorFusionSetConfigRequest *msg)
{
    (void)msg;
    AckResponse ack = AckResponse_init_zero;
    ack.success = true;
    protocol_tx_AckResponse(&ack);
}

/*---------------------------------------------------------------------------
 * Beacon (DATA beacon over UWB)
 *---------------------------------------------------------------------------*/
static uint16_t s_beacon_seq;

void protocol_rx_BeaconPingRequest(const BeaconPingRequest *msg)
{
    (void)msg;
    BeaconPingResponse r = BeaconPingResponse_init_zero;
    r.success = false;

    typedef struct
    {
        protocol_header_t header;
        uint16_t counter;
    } __attribute__((packed)) beacon_msg_t;

    beacon_msg_t b;
    b.header.protocol_type = (uint8_t)PROTOCOL_TYPE_DATA;
    b.header.msg_type      = (uint8_t)DATA_MSG_BEACON;
    b.header.sequence     = s_beacon_seq++;
    b.counter             = (uint16_t)(s_beacon_seq & 0xFFFF);

    uwb_send_result_t res = uwb_send_message((const uint8_t *)&b, sizeof(b), 0xFFFF);
    if (res.success)
        r.success = true;

    protocol_tx_BeaconPingResponse(&r);
}

/*---------------------------------------------------------------------------
 * Public Function Implementations (for exposing internal state)
 *---------------------------------------------------------------------------*/

bool protocol_handler_is_imu_streaming(void)
{
    return s_imu_stream_running;
}

void protocol_handler_set_response_destination(void)
{
    /* Route responses back to the same source that sent the request */
    protocol_uart_destination_e decoder_source = protocol_get_decoder_source();
    protocol_tx_destination_e dest = (decoder_source == PROTOCOL_UART_DEST_WIFI) 
        ? PROTOCOL_TX_DEST_UART_WIFI 
        : PROTOCOL_TX_DEST_UART_CONSOLE;
    protocol_tx_set_destination(dest);
}
