#pragma once

/*---------------------------------------------------------------------------
 * @file    node.h
 * @brief   Node configuration and identity management
 * @details Manages device identity, type, position, and capabilities.
 *          Separates application-level node concepts (tag/anchor) from
 *          protocol-level roles (initiator/responder in TWR, transmitter/
 *          receiver in TDOA, etc.)
 *---------------------------------------------------------------------------*/

#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

/**
 * @brief Node type classification
 * @note This represents the application-level role, not protocol role
 */
typedef enum
{
    NODE_TYPE_TAG,    // Mobile node (typically initiates ranging)
    NODE_TYPE_ANCHOR, // Fixed infrastructure node (typically responds)
    NODE_TYPE_HYBRID  // Can dynamically switch between roles
} node_type_e;

/**
 * @brief Node capabilities flags
 */
typedef struct
{
    bool supports_twr;  // Two-Way Ranging
    bool supports_tdoa; // Time Difference of Arrival (future)
} node_capabilities_t;

/**
 * @brief Complete node configuration
 */
typedef struct
{
    // Identity (address and PAN ID stored in UWB module as single source of truth)
    node_type_e type;

    // Position (for anchors with known locations)
    float position_x;
    float position_y;
    float position_z;
    bool position_known;

    // Capabilities
    node_capabilities_t capabilities;

    // Statistics
    uint32_t uptime_seconds;
    bool configured;
} node_config_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Set the node type (tag/anchor/hybrid)
 * @param type Node type
 */
void node_set_type(node_type_e type);

/**
 * @brief Get the current node type
 * @return Current node type
 */
node_type_e node_get_type(void);

/**
 * @brief Set the node position (for anchors)
 * @param position 3D position vector in meters
 */
void node_set_position(const vec3_t* position);

/**
 * @brief Get the node position
 * @param position Pointer to store position vector
 * @return true if position is known, false otherwise
 */
bool node_get_position(vec3_t* position);

/**
 * @brief Clear the node position (mark as unknown)
 */
void node_clear_position(void);

/**
 * @brief Get complete node configuration
 * @return Pointer to node configuration (read-only)
 */
const node_config_t* node_get_config(void);

/**
 * @brief Get node configuration status
 * @param config Pointer to store configuration copy
 */
void node_get_status(node_config_t* config);

/**
 * @brief Check if node is a tag
 * @return true if node type is TAG
 */
static inline bool node_is_tag(void)
{
    return node_get_type() == NODE_TYPE_TAG;
}

/**
 * @brief Check if node is an anchor
 * @return true if node type is ANCHOR
 */
static inline bool node_is_anchor(void)
{
    return node_get_type() == NODE_TYPE_ANCHOR;
}
