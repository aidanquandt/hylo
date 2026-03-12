/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu.h"
/* Suppress -Wtype-limits when IMU_NUM_DEVICES is 0 (loop condition i < 0 is always false for uint8_t i). */
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif
#include "FreeRTOS.h"
#include "common.h"
#include "counter.h"
#include "error_handler.h"
#include "imu_driver.h"
#include "module.h"
#include "gpio_driver.h"
#include "timer_driver.h"
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

/* Per-IMU context: device, state machine, and per-device state */
typedef struct
{
    imu_dev_t*         dev;
    bool               active;
    imu_data_t         raw_data;
    uint8_t            chip_id;
    imu_fault_code_e   fault_code;
    state_machine_s    state_machine;
} imu_ctx_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool imu_read_single(imu_ctx_t* ctx);
STATIC void imu_aggregate_and_push(void);
STATIC void imu_transform_accel(const vec3_t* accel_in, vec3_t* accel_out);
STATIC void imu_transform_gyro(const vec3_t* gyro_in, vec3_t* gyro_out);
STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void imu_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_state_active_process(void);
STATIC void imu_state_faulted_on_entry(uint16_t prevState);
STATIC imu_state_e imu_get_overall_state(void);

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

/* Current context index used by state machine callbacks (set before state_machine_periodic) */
STATIC volatile uint8_t imu_ctx_current_index = 0U;

/* Shared startup delay: all IMUs transition STARTUP -> INITIALIZATION when this reaches threshold */
STATIC uint32_t g_startup_delay_ticks = 0U;

/* Per-IMU contexts (sized by IMU_MAX_DEVICES to avoid zero-length array when IMU_NUM_DEVICES is 0). */
STATIC imu_ctx_t imu_ctxs[IMU_MAX_DEVICES];

/* Aggregate output for sensor fusion and public API: mean of accel/gyro/temp over all
 * IMUs currently in ACTIVE state. first_active_chip_id is the chip_id of the first
 * active IMU in the aggregation order (for backward-compatible status reporting). */
STATIC imu_data_t    aggregate_data        = {0};
STATIC uint32_t      aggregate_sample_count = 0U;
STATIC uint8_t       first_active_chip_id   = 0U;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC bool imu_read_single(imu_ctx_t* ctx)
{
    if (ctx == NULL || !ctx->active || ctx->dev == NULL)
    {
        return false;
    }

    vec3_t raw_accel = {0.0f, 0.0f, 0.0f};
    vec3_t raw_gyro  = {0.0f, 0.0f, 0.0f};

    if (imu_driver_read_accel_and_gyro(ctx->dev, &raw_accel, &raw_gyro) != IMU_DRIVER_SUCCESS)
    {
        return false;
    }

    imu_transform_accel(&raw_accel, &ctx->raw_data.accel);
    imu_transform_gyro(&raw_gyro, &ctx->raw_data.gyro);
    return true;
}

STATIC void imu_aggregate_and_push(void)
{
    vec3_t accel_sum = {0.0f, 0.0f, 0.0f};
    vec3_t gyro_sum  = {0.0f, 0.0f, 0.0f};
    float  temp_sum  = 0.0f;
    uint8_t count    = 0;
    bool   first     = true;

    if (IMU_NUM_DEVICES == 0U)
    {
        return;
    }
    for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
    {
        imu_ctx_t* c = &imu_ctxs[i];
        if (!c->active || c->state_machine.curr_state != IMU_STATE_ACTIVE)
        {
            continue;
        }
        accel_sum.x += c->raw_data.accel.x;
        accel_sum.y += c->raw_data.accel.y;
        accel_sum.z += c->raw_data.accel.z;
        gyro_sum.x  += c->raw_data.gyro.x;
        gyro_sum.y  += c->raw_data.gyro.y;
        gyro_sum.z  += c->raw_data.gyro.z;
        temp_sum    += c->raw_data.temperature;
        count++;
        if (first)
        {
            first_active_chip_id = c->chip_id;
            first = false;
        }
    }

    if (count == 0)
    {
        return;
    }

    float inv_n = 1.0f / (float)count;
    aggregate_data.accel.x = accel_sum.x * inv_n;
    aggregate_data.accel.y = accel_sum.y * inv_n;
    aggregate_data.accel.z = accel_sum.z * inv_n;
    aggregate_data.gyro.x  = gyro_sum.x  * inv_n;
    aggregate_data.gyro.y  = gyro_sum.y  * inv_n;
    aggregate_data.gyro.z  = gyro_sum.z  * inv_n;
    aggregate_data.temperature = temp_sum * inv_n;
}

STATIC void imu_transform_accel(const vec3_t* accel_in, vec3_t* accel_out)
{
    accel_out->x = accel_in->z;
    accel_out->y = accel_in->y;
    accel_out->z = -1.0f * accel_in->x;
}

STATIC void imu_transform_gyro(const vec3_t* gyro_in, vec3_t* gyro_out)
{
    gyro_out->x = gyro_in->z;
    gyro_out->y = gyro_in->y;
    gyro_out->z = -1.0f * gyro_in->x;
}

STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    uint8_t  idx       = imu_ctx_current_index;
    imu_ctx_t* ctx    = &imu_ctxs[idx];

    switch (currentState)
    {
        case IMU_STATE_STARTUP:
            if (g_startup_delay_ticks >= STARTUP_DELAY_MS)
            {
                nextState = IMU_STATE_INITIALIZATION;
            }
            break;

        case IMU_STATE_INITIALIZATION:
            if (ctx->fault_code != FAULT_NONE)
            {
                nextState = IMU_STATE_FAULTED;
            }
            else
            {
                nextState = IMU_STATE_ACTIVE;
            }
            break;

        case IMU_STATE_ACTIVE:
            if (ctx->fault_code != FAULT_NONE)
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

    uint8_t  idx = imu_ctx_current_index;
    imu_ctx_t* ctx = &imu_ctxs[idx];

    ctx->fault_code = FAULT_NONE;
    ctx->active     = false;
    memset(&ctx->raw_data, 0, sizeof(ctx->raw_data));
    ctx->chip_id = 0;
    ctx->dev    = imu_driver_init((imu_device_e)idx);
    if (ctx->dev == NULL)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: init returned NULL", idx);
        ctx->fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (imu_driver_probe_and_init(ctx->dev) != IMU_DRIVER_SUCCESS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: probe/init failed", idx);
        ctx->dev = NULL;
        ctx->fault_code = FAULT_PROBE_FAILED;
        return;
    }

    uint8_t chip_id = imu_driver_read_chip_id(ctx->dev);
    if (chip_id != IMU_EXPECTED_CHIP_ID_1 && chip_id != IMU_EXPECTED_CHIP_ID_2)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: invalid chip ID 0x%02X", idx,
                          chip_id);
        ctx->dev = NULL;
        ctx->fault_code = FAULT_PROBE_FAILED;
        return;
    }
    ctx->chip_id = chip_id;

    if (imu_driver_configure_accel(ctx->dev, IMU_ACCEL_RANGE_2G, IMU_ODR_200HZ) != IMU_DRIVER_SUCCESS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: accel config failed", idx);
        ctx->dev = NULL;
        ctx->fault_code = FAULT_PROBE_FAILED;
        return;
    }

    if (imu_driver_configure_gyro(ctx->dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_200HZ) != IMU_DRIVER_SUCCESS)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "imu", "IMU %u: gyro config failed", idx);
        ctx->dev = NULL;
        ctx->fault_code = FAULT_PROBE_FAILED;
        return;
    }

    ctx->active = true;
}

STATIC void imu_state_active_process(void)
{
    uint8_t  idx = imu_ctx_current_index;
    imu_ctx_t* ctx = &imu_ctxs[idx];

    if (!imu_read_single(ctx))
    {
        ctx->fault_code = FAULT_READ_FAILED;
    }
}

STATIC void imu_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;

    uint8_t  idx = imu_ctx_current_index;
    imu_ctx_t* ctx = &imu_ctxs[idx];

    const char* fault_str;
    switch (ctx->fault_code)
    {
        case FAULT_PROBE_FAILED:
            fault_str = "probe/init failed";
            break;
        case FAULT_READ_FAILED:
            fault_str = "sensor read failed";
            break;
        default:
            fault_str = "unknown fault";
            break;
    }

    error_handler_log(ERROR_SEVERITY_ERROR, "imu", "IMU %u: %s (code=%u)", idx, fault_str,
                      (uint8_t)ctx->fault_code);
}

STATIC imu_state_e imu_get_overall_state(void)
{
    bool any_startup = false;
    bool any_init    = false;
    bool any_active  = false;

    if (IMU_NUM_DEVICES == 0U)
    {
        return IMU_STATE_FAULTED;
    }
    for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
    {
        uint16_t s = imu_ctxs[i].state_machine.curr_state;
        switch (s)
        {
            case IMU_STATE_STARTUP:
                any_startup = true;
                break;
            case IMU_STATE_INITIALIZATION:
                any_init = true;
                break;
            case IMU_STATE_ACTIVE:
                any_active = true;
                break;
            case IMU_STATE_FAULTED:
                break;
            default:
                break;
        }
    }

    if (any_active)
    {
        return IMU_STATE_ACTIVE;
    }
    if (any_init || any_startup)
    {
        return any_init ? IMU_STATE_INITIALIZATION : IMU_STATE_STARTUP;
    }
    return IMU_STATE_FAULTED;
}

STATIC void imu_init(void)
{
    /* Initialize all slots to a known state (avoids uninitialized read when IMU_NUM_DEVICES is 0). */
    for (uint8_t i = 0; i < (uint8_t)IMU_MAX_DEVICES; i++)
    {
        imu_ctx_t* ctx = &imu_ctxs[i];
        memset(ctx, 0, sizeof(*ctx));
        ctx->state_machine.prev_state      = IMU_STATE_STARTUP;
        ctx->state_machine.curr_state      = IMU_STATE_STARTUP;
        ctx->state_machine.next_state      = IMU_STATE_STARTUP;
        ctx->state_machine.timer           = 0;
        ctx->state_machine.transitionLogic = imu_transition_logic;
        ctx->state_machine.states          = imu_states;
    }
}

STATIC void imu_process_1kHz(void)
{
    if (IMU_NUM_DEVICES == 0U)
    {
        return;
    }

    if (IMU_NUM_DEVICES > 0U)
    {
        if (g_startup_delay_ticks < STARTUP_DELAY_MS)
        {
            g_startup_delay_ticks++;
        }

        for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
        {
            imu_ctx_current_index = i;
            state_machine_periodic(&imu_ctxs[i].state_machine);
        }
    }

    /* Aggregate and push at 200 Hz; read temperatures at 1 Hz */
    static uint8_t  prescaler_avg  = 0;
    static uint16_t prescaler_temp = 0;

    if (counter_uint8_t(&prescaler_avg, IMU_ACCEL_GYRO_PRESCALER))
    {
        imu_aggregate_and_push();
        if (imu_get_overall_state() == IMU_STATE_ACTIVE)
        {
            sensor_event_t event = {
                .type         = SENSOR_EVENT_IMU,
                .timestamp_ms = timer_driver_get_time_ms(),
                .sequence     = aggregate_sample_count,
                .data.imu     = {
                    .accel_x = aggregate_data.accel.x,
                    .accel_y = aggregate_data.accel.y,
                    .accel_z = aggregate_data.accel.z,
                    .gyro_x  = aggregate_data.gyro.x,
                    .gyro_y  = aggregate_data.gyro.y,
                    .gyro_z  = aggregate_data.gyro.z,
                    .temp_c  = aggregate_data.temperature
                }
            };
            sensor_fusion_status_e sf_status = sensor_fusion_push_event(&event);
            if (sf_status != SENSOR_FUSION_SUCCESS)
            {
                error_handler_log(ERROR_SEVERITY_WARNING, "imu",
                                  "Failed to push IMU event to sensor fusion: %d", sf_status);
            }
            aggregate_sample_count++;
        }
    }

    if (counter_uint16_t(&prescaler_temp, IMU_TEMPERATURE_PRESCALER))
    {
        if (IMU_NUM_DEVICES > 0U)
        {
            for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
            {
                imu_ctx_t* c = &imu_ctxs[i];
                if (!c->active || c->dev == NULL)
                {
                    continue;
                }
                float t = imu_driver_read_temperature(c->dev);
                /* Only update on success; 0.0f is used by port to indicate read failure. */
                if (t != 0.0f)
                {
                    c->raw_data.temperature = t;
                }
            }
        }
        imu_aggregate_and_push();
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
/* Returns true if at least one active IMU accepted soft reset (any-success semantics). */
bool imu_soft_reset(void)
{
    if (imu_get_overall_state() != IMU_STATE_ACTIVE)
    {
        return false;
    }

    bool any_ok = false;
    if (IMU_NUM_DEVICES > 0U)
    {
        for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
        {
            imu_ctx_t* c = &imu_ctxs[i];
            if (c->active && c->dev != NULL)
            {
                if (imu_driver_soft_reset(c->dev) == IMU_DRIVER_SUCCESS)
                {
                    any_ok = true;
                }
            }
        }
    }
    return any_ok;
}

imu_state_e imu_get_state(imu_device_e device)
{
    if ((uint8_t)device >= (uint8_t)IMU_MAX_DEVICES || IMU_NUM_DEVICES == 0U)
    {
        return IMU_STATE_FAULTED;
    }
    return (imu_state_e)imu_ctxs[(uint8_t)device].state_machine.curr_state;
}

void imu_get_status(imu_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    status->state   = imu_get_overall_state();
    status->chip_id = first_active_chip_id; /* First active IMU at last aggregation; 0 if none */
    status->fault_code = 0U;
    if (IMU_NUM_DEVICES > 0U)
    {
        for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
        {
            if (imu_ctxs[i].state_machine.curr_state == IMU_STATE_FAULTED)
            {
                status->fault_code = (uint32_t)imu_ctxs[i].fault_code;
                break;
            }
        }
    }
}

bool imu_get_data(imu_data_t* data)
{
    if (data == NULL || imu_get_overall_state() != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *data = aggregate_data;
    return true;
}

bool imu_get_accel(vec3_t* accel)
{
    if (accel == NULL || imu_get_overall_state() != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *accel = aggregate_data.accel;
    return true;
}

bool imu_get_gyro(vec3_t* gyro)
{
    if (gyro == NULL || imu_get_overall_state() != IMU_STATE_ACTIVE)
    {
        return false;
    }

    *gyro = aggregate_data.gyro;
    return true;
}

bool imu_get_temp(imu_device_e device, float* temp)
{
    if (temp == NULL || (uint8_t)device >= (uint8_t)IMU_MAX_DEVICES || IMU_NUM_DEVICES == 0U)
    {
        return false;
    }
    if (imu_get_state(device) != IMU_STATE_ACTIVE)
    {
        return false;
    }

    uint8_t idx = (uint8_t)device;
    *temp = imu_ctxs[idx].raw_data.temperature;
    return imu_ctxs[idx].active;
}

bool imu_get_individual_data(imu_device_e device, imu_data_t* data)
{
    if (data == NULL || (uint8_t)device >= (uint8_t)IMU_MAX_DEVICES || IMU_NUM_DEVICES == 0U)
    {
        return false;
    }
    if (imu_get_state(device) != IMU_STATE_ACTIVE)
    {
        return false;
    }

    uint8_t idx = (uint8_t)device;
    *data = imu_ctxs[idx].raw_data;
    return imu_ctxs[idx].active;
}

uint8_t imu_get_device_count(void)
{
    return (uint8_t)IMU_NUM_DEVICES;
}

uint8_t imu_get_active_count(void)
{
    uint8_t n = 0;
    if (IMU_NUM_DEVICES > 0U)
    {
        for (uint8_t i = 0; i < (uint8_t)IMU_NUM_DEVICES; i++)
        {
            if (imu_ctxs[i].active && imu_ctxs[i].state_machine.curr_state == IMU_STATE_ACTIVE)
            {
                n++;
            }
        }
    }
    return n;
}
