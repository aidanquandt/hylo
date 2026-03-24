/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_node.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "system_halt.h"
#include "task_config.h"
#include "protocol_tx.h"
#include "protocol.pb.h"
#include "uwb.h"
#include <string.h>

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
STATIC void uwb_node_process_1Hz(void);
STATIC void uwb_node_1Hz_task(void* pvParameters);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void uwb_node_1Hz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        uwb_node_process_1Hz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    }
}

void uwb_node_init(void)
{
    if (module_initialized)
    {
        return;
    }

    uwb_node_config.configured = true;
    module_initialized         = true;

    BaseType_t result = xTaskCreate(uwb_node_1Hz_task, "uwb_node_1hz", TASK_STACK_2KB,
                                    NULL, TASK_PRIORITY_UWB_NODE, NULL);
    if (result != pdPASS)
    {
        system_halt("uwb_node", "Failed to create 1Hz task");
    }
}

STATIC void uwb_node_process_1Hz(void)
{
    if (module_initialized)
    {
        uwb_node_config.uptime_seconds++;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void uwb_node_set_type(uwb_node_type_e type)
{
    if (type > UWB_NODE_TYPE_HYBRID)
    {
        UwbNodeInvalidTypeEvent ev = UwbNodeInvalidTypeEvent_init_zero;
        ev.node_type = (int32_t)type;
        protocol_tx_UwbNodeInvalidTypeEvent(&ev);
        return;
    }

    uwb_node_config.type = type;

    UwbNodeTypeSetEvent ev = UwbNodeTypeSetEvent_init_zero;
    const char* type_str   = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                             : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                                              : "HYBRID";
    strncpy(ev.type_str, type_str, sizeof(ev.type_str) - 1);
    ev.type_str[sizeof(ev.type_str) - 1] = '\0';
    protocol_tx_UwbNodeTypeSetEvent(&ev);
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

    UwbNodePositionSetEvent ev = UwbNodePositionSetEvent_init_zero;
    ev.x = position->x;
    ev.y = position->y;
    ev.z = position->z;
    protocol_tx_UwbNodePositionSetEvent(&ev);
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
