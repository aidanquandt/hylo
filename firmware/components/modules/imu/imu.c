/*---------------------------------------------------------------------------
 * @file    imu.c
 * @brief   IMU hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu.h"
#include "error_handler.h"
#include "imu_port.h"
#include "module.h"
#include "platform_gpio.h"
#include "state_machine.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_EXPECTED_CHIP_ID_1 (0x43U)
#define IMU_EXPECTED_CHIP_ID_2 (0x44U)
#define STARTUP_DELAY_MS (2000U)

typedef enum
{
    STATE_STARTUP,
    STATE_INITIALIZATION,
    STATE_ACTIVE,
    STATE_FAULTED
} imu_state_E;

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
    bool fault_present;
    bool init_device_completed;
} imu_state_machine_inputs_t;

typedef struct
{
    imu_sensor_data_t accel;
    imu_sensor_data_t gyro;
    float temperature;
    uint8_t chip_id;
} imu_measurements_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void);
STATIC void read_sensors(void);
STATIC void imu_state_machine_sample_inputs(void);
STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void imu_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_state_active_process(void);
STATIC void imu_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void imu_init(void);
STATIC void imu_process_100Hz(void);

extern const module_S imu_module;

const module_S imu_module = {
    .module_name = "imu",
    .module_init = imu_init,
    .module_process_100Hz = imu_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC const state_s imu_states[] = {
    [STATE_STARTUP] = {.process = NULL, .onEntry = NULL, .onExit = NULL},
    [STATE_INITIALIZATION] = {.process = NULL,
                              .onEntry = imu_state_initialization_on_entry,
                              .onExit = NULL},
    [STATE_ACTIVE] = {.process = imu_state_active_process, .onEntry = NULL, .onExit = NULL},
    [STATE_FAULTED] = {.process = NULL, .onEntry = imu_state_faulted_on_entry, .onExit = NULL}};

STATIC state_machine_s imu_state_machine = {.prev_state = STATE_STARTUP,
                                            .curr_state = STATE_STARTUP,
                                            .next_state = STATE_STARTUP,
                                            .timer = 0,
                                            .transitionLogic = imu_transition_logic,
                                            .states = imu_states};

STATIC imu_dev_t* imu_dev = NULL;
STATIC imu_state_machine_inputs_t imu_state_machine_inputs = {0};
STATIC imu_measurements_t measurements = {0};
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

    measurements.chip_id = imu_port_read_chip_id(imu_dev);

    return (measurements.chip_id == IMU_EXPECTED_CHIP_ID_1 ||
            measurements.chip_id == IMU_EXPECTED_CHIP_ID_2);
}

STATIC void read_sensors(void)
{
    if (imu_dev == NULL)
    {
        imu_fault_code = FAULT_READ_FAILED;
        return;
    }

    measurements.temperature = imu_port_read_temperature(imu_dev);

    int result = imu_port_read_accel_and_gyro(imu_dev, &measurements.accel, &measurements.gyro);
    if (result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_READ_FAILED;
    }
}

STATIC void imu_init(void)
{
}

STATIC void imu_process_100Hz(void)
{
    imu_state_machine_sample_inputs();
    state_machine_periodic(&imu_state_machine);
}

STATIC void imu_state_machine_sample_inputs(void)
{
    imu_state_machine_inputs.fault_present = (imu_fault_code != FAULT_NONE);
    imu_state_machine_inputs.init_device_completed = (imu_dev != NULL);
}

STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;

    switch (currentState)
    {
        case STATE_STARTUP:
            if (stateTimer >= MS_TO_100HZ_TICKS(STARTUP_DELAY_MS))
            {
                nextState = STATE_INITIALIZATION;
            }
            break;

        case STATE_INITIALIZATION:
            if (imu_state_machine_inputs.fault_present)
            {
                nextState = STATE_FAULTED;
            }
            else
            {
                nextState = STATE_ACTIVE;
            }
            break;

        case STATE_ACTIVE:
            if (imu_state_machine_inputs.fault_present)
            {
                nextState = STATE_FAULTED;
            }
            break;

        case STATE_FAULTED:
            nextState = STATE_FAULTED;
            break;

        default:
            nextState = STATE_FAULTED;
            break;
    }

    return nextState;
}

STATIC void imu_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;

    imu_fault_code = FAULT_NONE;

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

    int accel_result = imu_port_configure_accel(imu_dev, IMU_ACCEL_RANGE_2G, IMU_ODR_100HZ);
    if (accel_result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_ACCEL_CONFIG_FAILED;
        return;
    }

    int gyro_result = imu_port_configure_gyro(imu_dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_100HZ);
    if (gyro_result != IMU_PORT_SUCCESS)
    {
        imu_fault_code = FAULT_GYRO_CONFIG_FAILED;
        return;
    }
}

STATIC void imu_state_active_process(void)
{
    read_sensors();
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
    if (imu_dev == NULL || imu_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    imu_port_status_t ret = imu_port_soft_reset(imu_dev);
    return (ret == IMU_PORT_SUCCESS);
}

/*---------------------------------------------------------------------------
 * Public API Implementation
 *---------------------------------------------------------------------------*/

void imu_get_status(imu_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    switch (imu_state_machine.curr_state)
    {
        case STATE_ACTIVE:
            status->state = IMU_STATE_ACTIVE;
            break;
        case STATE_INITIALIZATION:
            status->state = IMU_STATE_INITIALIZATION;
            break;
        case STATE_FAULTED:
            status->state = IMU_STATE_FAULTED;
            break;
        default:
            status->state = IMU_STATE_STARTUP;
            break;
    }

    status->chip_id = measurements.chip_id;
    status->fault_code = imu_fault_code;
}

bool imu_get_data(imu_data_t* data)
{
    if (data == NULL || imu_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    data->accel.x = measurements.accel.x;
    data->accel.y = measurements.accel.y;
    data->accel.z = measurements.accel.z;
    data->gyro.x = measurements.gyro.x;
    data->gyro.y = measurements.gyro.y;
    data->gyro.z = measurements.gyro.z;
    data->temperature = measurements.temperature;
    return true;
}

bool imu_get_accel(imu_vector3_t* accel)
{
    if (accel == NULL || imu_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    accel->x = measurements.accel.x;
    accel->y = measurements.accel.y;
    accel->z = measurements.accel.z;
    return true;
}

bool imu_get_gyro(imu_vector3_t* gyro)
{
    if (gyro == NULL || imu_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    gyro->x = measurements.gyro.x;
    gyro->y = measurements.gyro.y;
    gyro->z = measurements.gyro.z;
    return true;
}

bool imu_get_temp(float* temp)
{
    if (temp == NULL || imu_state_machine.curr_state != STATE_ACTIVE)
    {
        return false;
    }

    *temp = measurements.temperature;
    return true;
}
