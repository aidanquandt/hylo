/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_node.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "module.h"
#include "task.h"
#include "task_config.h"
#include "uwb.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uwb_node_config_t uwb_node_config = {
    .type           = UWB_NODE_TYPE_TAG, // Default to tag
    .position_x     = 0.0f,
    .position_y     = 0.0f,
    .position_z     = 0.0f,
    .position_known = false,
    .capabilities   = {.supports_twr = true, .supports_tdoa = false},
    .uptime_seconds = 0,
    .configured     = false};

STATIC bool module_initialized = false;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void uwb_node_module_init(void);
STATIC void uwb_node_create_tasks(void);
STATIC void uwb_node_task(void* argument);

/*---------------------------------------------------------------------------
 * Module Registration
 *---------------------------------------------------------------------------*/
extern const module_S uwb_node_module;

const module_S uwb_node_module = {
    .module_name         = "uwb_node",
    .module_init         = uwb_node_module_init,
    .module_create_tasks = uwb_node_create_tasks,
};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void uwb_node_module_init(void)
{
    if (module_initialized)
    {
        return;
    }

    uwb_node_config.configured = true;
    module_initialized         = true;
}

STATIC void uwb_node_create_tasks(void)
{
    BaseType_t result = xTaskCreate(uwb_node_task, "uwb_node", TASK_STACK_SMALL, NULL,
                                    TASK_PRIORITY_UWB_NODE, NULL);
    if (result != pdPASS)
    {
        error_handler_fatal("uwb_node", "Failed to create uwb_node task");
    }
}

/**
 * @brief UWB node task - manages node state at 1Hz
 */
STATIC void uwb_node_task(void* argument)
{
    (void)argument;

    TickType_t lastWake     = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000); // 1Hz = 1000ms

    watchdog_register_task(2000); // Expect heartbeat every 2 seconds

    for (;;)
    {
        if (module_initialized)
        {
            uwb_node_config.uptime_seconds++;
        }

        watchdog_heartbeat();
        vTaskDelayUntil(&lastWake, period);
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void uwb_node_set_type(uwb_node_type_e type)
{
    if (type > UWB_NODE_TYPE_HYBRID)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_node", "Invalid node type: %d", type);
        return;
    }

    uwb_node_config.type = type;

    const char* type_str = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                           : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                                            : "HYBRID";
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_node", "Type set to: %s", type_str);
}

uwb_node_type_e uwb_node_get_type(void)
{
    return uwb_node_config.type;
}

void uwb_node_set_position(const vec3_t* position)
{
    if (position == NULL)
    {
        return;
    }

    uwb_node_config.position_x     = position->x;
    uwb_node_config.position_y     = position->y;
    uwb_node_config.position_z     = position->z;
    uwb_node_config.position_known = true;

    error_handler_log(ERROR_SEVERITY_INFO, "uwb_node", "Position set: (%.2f, %.2f, %.2f)",
                      position->x, position->y, position->z);
}

bool uwb_node_get_position(vec3_t* position)
{
    if (!uwb_node_config.position_known || position == NULL)
    {
        return false;
    }

    position->x = uwb_node_config.position_x;
    position->y = uwb_node_config.position_y;
    position->z = uwb_node_config.position_z;

    return true;
}

void uwb_node_clear_position(void)
{
    uwb_node_config.position_known = false;
    uwb_node_config.position_x     = 0.0f;
    uwb_node_config.position_y     = 0.0f;
    uwb_node_config.position_z     = 0.0f;
}

const uwb_node_config_t* uwb_node_get_config(void)
{
    return &uwb_node_config;
}

void uwb_node_get_status(uwb_node_config_t* config)
{
    if (config != NULL)
    {
        memcpy(config, &uwb_node_config, sizeof(uwb_node_config_t));
    }
}
