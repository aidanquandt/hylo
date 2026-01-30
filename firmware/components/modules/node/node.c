/*---------------------------------------------------------------------------
 * @file    node.c
 * @brief   Node configuration and identity management implementation
 *---------------------------------------------------------------------------*/

#include "node.h"
#include "error_handler.h"
#include "module.h"
#include "uwb.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC node_config_t node_config = {.type           = NODE_TYPE_TAG, // Default to tag
                                    .position_x     = 0.0f,
                                    .position_y     = 0.0f,
                                    .position_z     = 0.0f,
                                    .position_known = false,
                                    .capabilities = {.supports_twr = true, .supports_tdoa = false},
                                    .uptime_seconds = 0,
                                    .configured     = false};

STATIC bool module_initialized = false;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void node_module_init(void);
STATIC void node_process_1Hz(void);
STATIC void node_process_10Hz(void);

/*---------------------------------------------------------------------------
 * Module Registration
 *---------------------------------------------------------------------------*/
extern const module_S node_module;

const module_S node_module = {
    .module_name         = "node",
    .module_init         = node_module_init,
    .module_process_1Hz  = node_process_1Hz,
    .module_process_10Hz = node_process_10Hz,
};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void node_module_init(void)
{
    if (module_initialized)
    {
        return;
    }

    node_config.configured = true;
    module_initialized     = true;
}

STATIC void node_process_1Hz(void)
{
    if (module_initialized)
    {
        node_config.uptime_seconds++;
    }
}

STATIC void node_process_10Hz(void)
{
    STATIC bool uwb_sync_completed = false;

    // Once UWB is ready, ensure default address is set
    if (!uwb_sync_completed && uwb_is_ready())
    {
        // Set default address if UWB doesn't have one yet
        if (uwb_get_address() == 0x0000)
        {
            uwb_set_address(0x0001, 0xDECA); // Default PAN ID
        }
        uwb_sync_completed = true;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void node_set_type(node_type_e type)
{
    if (type > NODE_TYPE_HYBRID)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "node", "Invalid node type: %d", type);
        return;
    }

    node_config.type = type;

    const char* type_str = (type == NODE_TYPE_TAG)      ? "TAG"
                           : (type == NODE_TYPE_ANCHOR) ? "ANCHOR"
                                                        : "HYBRID";
    error_handler_log(ERROR_SEVERITY_INFO, "node", "Type set to: %s", type_str);
}

node_type_e node_get_type(void)
{
    return node_config.type;
}

void node_set_position(const vec3_t* position)
{
    if (position == NULL)
    {
        return;
    }

    node_config.position_x     = position->x;
    node_config.position_y     = position->y;
    node_config.position_z     = position->z;
    node_config.position_known = true;

    error_handler_log(ERROR_SEVERITY_INFO, "node", "Position set: (%.2f, %.2f, %.2f)", position->x,
                      position->y, position->z);
}

bool node_get_position(vec3_t* position)
{
    if (!node_config.position_known || position == NULL)
    {
        return false;
    }

    position->x = node_config.position_x;
    position->y = node_config.position_y;
    position->z = node_config.position_z;

    return true;
}

void node_clear_position(void)
{
    node_config.position_known = false;
    node_config.position_x     = 0.0f;
    node_config.position_y     = 0.0f;
    node_config.position_z     = 0.0f;
}

const node_config_t* node_get_config(void)
{
    return &node_config;
}

void node_get_status(node_config_t* config)
{
    if (config != NULL)
    {
        memcpy(config, &node_config, sizeof(node_config_t));
    }
}
