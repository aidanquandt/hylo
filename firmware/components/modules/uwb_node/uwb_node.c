/*---------------------------------------------------------------------------
 * @file    uwb_node.c
 * @brief   UWB Node configuration and identity management implementation
 *---------------------------------------------------------------------------*/

#include "uwb_node.h"
#include "error_handler.h"
#include "module.h"
#include "uwb.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uwb_node_config_t uwb_node_config = {.type           = UWB_NODE_TYPE_TAG, // Default to tag
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
STATIC void uwb_node_module_init(void);
STATIC void uwb_node_process_1Hz(void);
STATIC void uwb_node_process_10Hz(void);

/*---------------------------------------------------------------------------
 * Module Registration
 *---------------------------------------------------------------------------*/
extern const module_S uwb_node_module;

const module_S uwb_node_module = {
    .module_name         = "uwb_node",
    .module_init         = uwb_node_module_init,
    .module_process_1Hz  = uwb_node_process_1Hz,
    .module_process_10Hz = uwb_node_process_10Hz,
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
    module_initialized     = true;
}

STATIC void uwb_node_process_1Hz(void)
{
    if (module_initialized)
    {
        uwb_node_config.uptime_seconds++;
    }
}

STATIC void uwb_node_process_10Hz(void)
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

    error_handler_log(ERROR_SEVERITY_INFO, "uwb_node", "Position set: (%.2f, %.2f, %.2f)", position->x,
                      position->y, position->z);
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
