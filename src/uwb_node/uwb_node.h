#pragma once
/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UWB_NODE_TYPE_TAG,    // Mobile node (typically initiates ranging)
    UWB_NODE_TYPE_ANCHOR, // Fixed infrastructure node (typically responds)
    UWB_NODE_TYPE_HYBRID  // Can dynamically switch between roles
} uwb_node_type_e;

typedef struct
{
    bool supports_twr;  // Two-Way Ranging
    bool supports_tdoa; // Time Difference of Arrival (future)
} uwb_node_capabilities_t;

typedef struct
{
    // Identity (address and PAN ID stored in UWB module as single source of truth)
    uwb_node_type_e type;

    // Position (for anchors with known locations)
    float position_x;
    float position_y;
    float position_z;
    bool position_known;

    // Capabilities
    uwb_node_capabilities_t capabilities;

    bool configured;
} uwb_node_config_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void uwb_node_init(void);
void uwb_node_set_type(uwb_node_type_e type);
uwb_node_type_e uwb_node_get_type(void);
void uwb_node_set_position(const vec3_t* position);
bool uwb_node_get_position(vec3_t* position);
void uwb_node_clear_position(void);
const uwb_node_config_t* uwb_node_get_config(void);
void uwb_node_get_status(uwb_node_config_t* config);

static inline bool uwb_node_is_tag(void)
{
    return uwb_node_get_type() == UWB_NODE_TYPE_TAG;
}

static inline bool uwb_node_is_anchor(void)
{
    return uwb_node_get_type() == UWB_NODE_TYPE_ANCHOR;
}
