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
#include "platform_timer.h"
#include "sensor_fusion.h"
#include "state_machine.h"
#include "task.h"


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
    FAULT_PROBE_FAILED,
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
STATIC bool imu_read_single(uint8_t i);
STATIC bool imu_compute_average(void);
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
STATIC void imu_process_1kHz(void);

const module_S imu_module = {
    .module_name         = "imu",
    .module_init         = imu_init,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = imu_process_1kHz,
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

STATIC imu_dev_t* imu_devs[IMU_NUM_DEVICES]     = {NULL};
STATIC bool imu_dev_active[IMU_NUM_DEVICES]      = {false};
STATIC imu_data_t imu_raw_data[IMU_NUM_DEVICES]  = {0};
STATIC uint8_t imu_active_count                  = 0;
STATIC imu_ctx_t ctx                             = {0};
STATIC imu_fault_code_e imu_fault_code           = FAULT_NONE;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
/* Read one IMU by index, update its cached raw_data entry. */
STATIC bool imu_read_single(uint8_t i)
{
    if (!imu_dev_active[i] || imu_devs[i] == NULL)
    {
        return false;
    }

    vec3_t raw_accel = {0.0f, 0.0f, 0.0f};
    vec3_t raw_gyro  = {0.0f, 0.0f, 0.0f};

    if (imu_port_read_accel_and_gyro(imu_devs[i], &raw_accel, &raw_gyro) != IMU_PORT_SUCCESS)
    {
        return false;
    }

    imu_transform_accel(&raw_accel, &imu_raw_data[i].accel);
    imu_transform_gyro(&raw_gyro, &imu_raw_data[i].gyro);
    return true;
}

/* Recompute ctx.data average from all cached raw_data entries. */
STATIC bool imu_compute_average(void)
{
    vec3_t accel_sum = {0.0f, 0.0f, 0.0f};
    vec3_t gyro_sum  = {0.0f, 0.0f, 0.0f};
    uint8_t count    = 0;

    for (uint8_t i = 0; i < IMU_NUM_DEVICES; i++)
    {
        if (!imu_dev_active[i])
        {
            continue;
        }
        accel_sum.x += imu_raw_data[i].accel.x;
        accel_sum.y += imu_raw_data[i].accel.y;
        accel_sum.z += imu_raw_data[i].accel.z;
        gyro_sum.x  += imu_raw_data[i].gyro.x;
        gyro_sum.y  += imu_raw_data[i].gyro.y;
        gyro_sum.z  += imu_raw_data[i].gyro.z;
        count++;
    }

    if (count == 0)
    {
        return false;
    }

    float inv_n      = 1.0f / (float)count;
    ctx.data.accel.x = accel_sum.x * inv_n;
    ctx.data.accel.y = accel_sum.y * inv_n;
    ctx.data.accel.z = accel_sum.z * inv_n;
    ctx.data.gyro.x  = gyro_sum.x  * inv_n;
    ctx.data.gyro.y  = gyro_sum.y  * inv_n;
    ctx.data.gyro.z  = gyro_sum.z  * inv_n;
    return true;
}

STATIC void imu_read_temperature(void)
{
    float temp_sum = 0.0f;
    uint8_t count  = 0;

    for (uint8_t i = 0; i < IMU_NUM_DEVICES; i++)
    {
        if (!imu_dev_active[i] || imu_devs[i] == NULL)
        {
            continue;
        }

        float t = imu_port_read_temperature(imu_devs[i]);
        if (t != 0.0f)
        {
            imu_raw_data[i].temperature = t;
            temp_sum += t;
            count++;
        }
    }

    if (count > 0)
    {
        ctx.data.temperature = temp_sum / (float)count;
    }
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
                            .timestamp_ms = platform_get_time_ms(),
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

STATIC void imu_process_1kHz(void)
{
    imu_state_machine_sample_inputs();
    state_machine_periodic(&imu_state_machine);
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
    imu_active_count = 0;
    memset(imu_dev_active, 0, sizeof(imu_dev_active));
    memset(imu_raw_data, 0, sizeof(imu_raw_data));

    for (uint8_t i = 0; i < IMU_NUM_DEVICES; i++)
    {
        imu_devs[i] = imu_port_init(i);
        if (imu_devs[i] == NULL)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: init returned NULL", i);
            continue;
        }

        if (imu_port_probe_and_init(imu_devs[i]) != IMU_PORT_SUCCESS)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: probe/init failed", i);
            imu_devs[i] = NULL;
            continue;
        }

        uint8_t chip_id = imu_port_read_chip_id(imu_devs[i]);
        if (chip_id != IMU_EXPECTED_CHIP_ID_1 && chip_id != IMU_EXPECTED_CHIP_ID_2)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: invalid chip ID 0x%02X", i,
                              chip_id);
            imu_devs[i] = NULL;
            continue;
        }

        if (imu_active_count == 0)
        {
            ctx.chip_id = chip_id; // expose first active chip_id for backward compat
        }

        if (imu_port_configure_accel(imu_devs[i], IMU_ACCEL_RANGE_2G, IMU_ODR_200HZ) !=
            IMU_PORT_SUCCESS)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: accel config failed", i);
            imu_devs[i] = NULL;
            continue;
        }

        if (imu_port_configure_gyro(imu_devs[i], IMU_GYRO_RANGE_2000DPS, IMU_ODR_200HZ) !=
            IMU_PORT_SUCCESS)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: gyro config failed", i);
            imu_devs[i] = NULL;
            continue;
        }

        imu_dev_active[i] = true;
        imu_active_count++;
    }

    if (imu_active_count == 0)
    {
        imu_fault_code = FAULT_PROBE_FAILED;
    }
}

STATIC void imu_state_active_process(void)
{
    /* Round-robin: read exactly one active IMU per 1kHz tick so that all 4
     * SPI transactions are spread across 4 consecutive ticks instead of
     * being serialised in a single tick (which exceeded the 1ms deadline). */
    static uint8_t  rr_index              = 0;
    static uint8_t  prescaler_counter_avg = 0;
    static uint16_t prescaler_counter_temp = 0;

    /* Advance round-robin index, skipping inactive devices. */
    for (uint8_t attempt = 0; attempt < IMU_NUM_DEVICES; attempt++)
    {
        uint8_t i = rr_index;
        rr_index  = (rr_index + 1U) % IMU_NUM_DEVICES;
        if (imu_dev_active[i])
        {
            imu_read_single(i);
            break;
        }
    }

    /* Push averaged data to sensor fusion at IMU_ACCEL_GYRO_READ_FREQ_HZ. */
    if (counter_uint8_t(&prescaler_counter_avg, IMU_ACCEL_GYRO_PRESCALER))
    {
        if (imu_compute_average())
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
        case FAULT_PROBE_FAILED:
            fault_str = "All IMUs probe/init failed";
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
    if (imu_state_machine.curr_state != IMU_STATE_ACTIVE || imu_active_count == 0)
    {
        return false;
    }

    bool any_ok = false;
    for (uint8_t i = 0; i < IMU_NUM_DEVICES; i++)
    {
        if (imu_dev_active[i] && imu_devs[i] != NULL)
        {
            if (imu_port_soft_reset(imu_devs[i]) == IMU_PORT_SUCCESS)
            {
                any_ok = true;
            }
        }
    }
    return any_ok;
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

bool imu_get_individual_data(uint8_t index, imu_data_t* data)
{
    if (data == NULL || index >= IMU_NUM_DEVICES ||
        imu_state_machine.curr_state != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *data = imu_raw_data[index];
    return imu_dev_active[index];
}

uint8_t imu_get_active_count(void)
{
    return imu_active_count;
}
