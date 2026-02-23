/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu.h"
#include "FreeRTOS.h"
#include "common.h"
#include "counter.h"
#include "error_handler.h"
#include "imu_port.h"
#include "module.h"
#include "platform_gpio.h"
#include "sensor_fusion.h"
#include "state_machine.h"
#include "task.h"
#include "task_config.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_EXPECTED_CHIP_ID_1 (0x43U)
#define IMU_EXPECTED_CHIP_ID_2 (0x44U)
#define STARTUP_DELAY_MS (2000U)
#define IMU_PROCESS_FREQ_HZ (1000U)
#define IMU_ACCEL_GYRO_READ_FREQ_HZ (200U)
#define IMU_TEMPERATURE_READ_FREQ_HZ (1U)
#define IMU_ACCEL_GYRO_PRESCALER (IMU_PROCESS_FREQ_HZ / IMU_ACCEL_GYRO_READ_FREQ_HZ)
#define IMU_TEMPERATURE_PRESCALER (IMU_PROCESS_FREQ_HZ / IMU_TEMPERATURE_READ_FREQ_HZ)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    FAULT_NONE = 0,
    FAULT_INIT_NULL_DEV,
    FAULT_PROBE_FAILED,
    FAULT_CHIP_ID_INVALID,
    FAULT_ACCEL_CONFIG_FAILED,
    FAULT_GYRO_CONFIG_FAILED,
    FAULT_READ_FAILED
} imu_fault_code_e;

typedef struct
{
    imu_data_t data;       // Public sensor data (accel, gyro, temperature)
    uint8_t chip_id;       // Chip ID from device
    uint32_t sample_count; // Total samples pushed to sensor fusion
} imu_ctx_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void);
STATIC bool imu_read_accel_and_gyro(void);
STATIC void imu_read_temperature(void);
STATIC void imu_push_to_sensor_fusion(void);
STATIC void imu_transform_accel(const vec3_t* accel_in, vec3_t* accel_out);
STATIC void imu_transform_gyro(const vec3_t* gyro_in, vec3_t* gyro_out);
STATIC void imu_state_machine_sample_inputs(void);
STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void imu_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_state_active_process(void);
STATIC void imu_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void imu_init(void);
STATIC void imu_create_tasks(void);
STATIC void imu_task(void* argument);

const module_S imu_module = {
    .module_name         = "imu",
    .module_init         = imu_init,
    .module_create_tasks = imu_create_tasks,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC const state_s imu_states[] = {
    [IMU_STATE_STARTUP]        = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [IMU_STATE_INITIALIZATION] = {.process = NULL,
                                  .onEntry = imu_state_initialization_on_entry,
                                  .onExit  = NULL},
    [IMU_STATE_ACTIVE]  = {.process = imu_state_active_process, .onEntry = NULL, .onExit = NULL},
    [IMU_STATE_FAULTED] = {.process = NULL, .onEntry = imu_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s imu_state_machine = {.prev_state      = IMU_STATE_STARTUP,
                                            .curr_state      = IMU_STATE_STARTUP,
                                            .next_state      = IMU_STATE_STARTUP,
                                            .timer           = 0,
                                            .transitionLogic = imu_transition_logic,
                                            .states          = imu_states};

STATIC imu_dev_t* imu_dev              = NULL;
STATIC imu_ctx_t ctx                   = {0};
STATIC imu_fault_code_e imu_fault_code = FAULT_NONE;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void)
{
    if (imu_dev == NULL)
    {
        return false;
    }

    int ret = imu_port_check_device_id(imu_dev);
    if (ret != IMU_PORT_SUCCESS)
    {
        return false;
    }

    ctx.chip_id = imu_port_read_chip_id(imu_dev);

    return (ctx.chip_id == IMU_EXPECTED_CHIP_ID_1 || ctx.chip_id == IMU_EXPECTED_CHIP_ID_2);
}

STATIC bool imu_read_accel_and_gyro(void)
{
    if (imu_dev == NULL)
    {
        imu_fault_code = FAULT_READ_FAILED;
        return false;
    }

    vec3_t accel_pre_transform;
    vec3_t gyro_pre_transform;

    int result = imu_port_read_accel_and_gyro(imu_dev, &accel_pre_transform, &gyro_pre_transform);

    if (result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_READ_FAILED;
        return false;
    }

    imu_transform_accel(&accel_pre_transform, &ctx.data.accel);
    imu_transform_gyro(&gyro_pre_transform, &ctx.data.gyro);

    return true;
}

STATIC void imu_read_temperature(void)
{
    if (imu_dev == NULL)
    {
        imu_fault_code = FAULT_READ_FAILED;
        return;
    }

    ctx.data.temperature = imu_port_read_temperature(imu_dev);
}

STATIC void imu_transform_accel(const vec3_t* accel_in, vec3_t* accel_out)
{
    // Identity mapping (sensor frame = body frame)
    accel_out->x = accel_in->z;
    accel_out->y = accel_in->y;
    accel_out->z = -1 * accel_in->x;
}

STATIC void imu_transform_gyro(const vec3_t* gyro_in, vec3_t* gyro_out)
{
    gyro_out->x = gyro_in->z;
    gyro_out->y = gyro_in->y;
    gyro_out->z = -1 * gyro_in->x;
}

STATIC void imu_push_to_sensor_fusion(void)
{
    sensor_event_t event = {.type         = SENSOR_EVENT_IMU,
                            .timestamp_ms = xTaskGetTickCount(),
                            .sequence     = ctx.sample_count,
                            .data.imu     = {.accel_x = ctx.data.accel.x,
                                             .accel_y = ctx.data.accel.y,
                                             .accel_z = ctx.data.accel.z,
                                             .gyro_x  = ctx.data.gyro.x,
                                             .gyro_y  = ctx.data.gyro.y,
                                             .gyro_z  = ctx.data.gyro.z,
                                             .temp_c  = ctx.data.temperature}};

    sensor_fusion_status_e sf_status = sensor_fusion_push_event(&event);
    if (sf_status != SENSOR_FUSION_SUCCESS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu",
                          "Failed to push IMU event to sensor fusion: %d", sf_status);
    }
}

STATIC void imu_init(void)
{
    // Initialization handled by state machine on first process call
}

STATIC void imu_create_tasks(void)
{
    BaseType_t result =
        xTaskCreate(imu_task, "imu", TASK_STACK_MEDIUM, NULL, TASK_PRIORITY_IMU_PERIODIC, NULL);
    if (result != pdPASS)
    {
        error_handler_fatal("imu", "Failed to create IMU task");
    }
}

/**
 * @brief IMU processing task - runs state machine at 1kHz
 */
STATIC void imu_task(void* argument)
{
    (void)argument;

    TickType_t lastWake     = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1); // 1kHz = 1ms

    watchdog_register_task(10); // Expect heartbeat every 10ms

    for (;;)
    {
        imu_state_machine_sample_inputs();
        state_machine_periodic(&imu_state_machine);

        watchdog_heartbeat();
        vTaskDelayUntil(&lastWake, period);
    }
}

STATIC void imu_state_machine_sample_inputs(void)
{
    // State machine input sampling (currently unused, kept for future expansion)
}

STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;

    switch (currentState)
    {
        case IMU_STATE_STARTUP:
            if (stateTimer >= MS_TO_1KHZ_TICKS(STARTUP_DELAY_MS))
            {
                nextState = IMU_STATE_INITIALIZATION;
            }
            break;

        case IMU_STATE_INITIALIZATION:
            if (imu_fault_code != FAULT_NONE)
            {
                nextState = IMU_STATE_FAULTED;
            }
            else
            {
                nextState = IMU_STATE_ACTIVE;
            }
            break;

        case IMU_STATE_ACTIVE:
            if (imu_fault_code != FAULT_NONE)
            {
                nextState = IMU_STATE_FAULTED;
            }
            break;

        case IMU_STATE_FAULTED:
            nextState = IMU_STATE_FAULTED;
            break;

        default:
            nextState = IMU_STATE_FAULTED;
            break;
    }

    return nextState;
}

STATIC void imu_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;

    imu_fault_code   = FAULT_NONE;
    ctx.sample_count = 0;

    imu_dev = imu_port_init();
    if (imu_dev == NULL)
    {
        imu_fault_code = FAULT_INIT_NULL_DEV;
        return;
    }

    int probe_result = imu_port_probe_and_init(imu_dev);
    if (probe_result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (!verify_chip_id())
    {
        imu_fault_code = FAULT_CHIP_ID_INVALID;
        return;
    }

    int accel_result = imu_port_configure_accel(imu_dev, IMU_ACCEL_RANGE_2G, IMU_ODR_200HZ);
    if (accel_result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_ACCEL_CONFIG_FAILED;
        return;
    }

    int gyro_result = imu_port_configure_gyro(imu_dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_200HZ);
    if (gyro_result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_GYRO_CONFIG_FAILED;
        return;
    }
}

STATIC void imu_state_active_process(void)
{
    static uint8_t prescaler_counter_accel_gyro = 0;
    static uint16_t prescaler_counter_temp      = 0;

    if (counter_uint8_t(&prescaler_counter_accel_gyro, IMU_ACCEL_GYRO_PRESCALER))
    {
        if (imu_read_accel_and_gyro())
        {
            imu_push_to_sensor_fusion();
            ctx.sample_count++;
        }
    }

    if (counter_uint16_t(&prescaler_counter_temp, IMU_TEMPERATURE_PRESCALER))
    {
        imu_read_temperature();
    }
}

STATIC void imu_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    const char* fault_str;
    switch (imu_fault_code)
    {
        case FAULT_INIT_NULL_DEV:
            fault_str = "Device init failed (NULL)";
            break;
        case FAULT_PROBE_FAILED:
            fault_str = "Probe/init failed";
            break;
        case FAULT_CHIP_ID_INVALID:
            fault_str = "Invalid chip ID";
            break;
        case FAULT_ACCEL_CONFIG_FAILED:
            fault_str = "Accel config failed";
            break;
        case FAULT_GYRO_CONFIG_FAILED:
            fault_str = "Gyro config failed";
            break;
        case FAULT_READ_FAILED:
            fault_str = "Sensor read failed";
            break;
        default:
            fault_str = "Unknown fault";
            break;
    }

    error_handler_log(ERROR_SEVERITY_ERROR, "imu", "%s (code=%u)", fault_str, imu_fault_code);
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
bool imu_soft_reset(void)
{
    if (imu_dev == NULL || imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    imu_port_status_t ret = imu_port_soft_reset(imu_dev);
    return (ret == IMU_PORT_SUCCESS);
}

void imu_get_status(imu_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->state      = (imu_state_e)imu_state_machine.curr_state;
    status->chip_id    = ctx.chip_id;
    status->fault_code = imu_fault_code;
}

bool imu_get_data(imu_data_t* data)
{
    if (data == NULL || imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *data = ctx.data;
    return true;
}

bool imu_get_accel(vec3_t* accel)
{
    if (accel == NULL || imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *accel = ctx.data.accel;
    return true;
}

bool imu_get_gyro(vec3_t* gyro)
{
    if (gyro == NULL || imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *gyro = ctx.data.gyro;
    return true;
}

bool imu_get_temp(float* temp)
{
    if (temp == NULL || imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *temp = ctx.data.temperature;
    return true;
}
