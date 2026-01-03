/*---------------------------------------------------------------------------
 * @file    test_beacon.h
 * @brief   Test beacon module - listens for UWB messages and responds
 *
 * @note    This is demo/test code that uses the uwb.c API. It listens for
 *          incoming UWB messages and responds with an auto-incrementing counter.
 *          This demonstrates how application modules should use the UWB
 *          infrastructure. Can be excluded from production builds.
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Beacon operating mode
 */
typedef enum
{
    BEACON_MODE_RESPONDER, ///< Auto-respond to incoming messages
    BEACON_MODE_MASTER     ///< Only listen, don't auto-respond
} beacon_mode_e;

/**
 * @brief Beacon status information
 */
typedef struct
{
    beacon_mode_e mode;     ///< Current operating mode
    uint16_t counter;       ///< Current counter value
    uint32_t rx_count;      ///< Messages received
    uint32_t tx_count;      ///< Responses sent
    uint16_t last_src_addr; ///< Last sender's address
} beacon_status_t;

/**
 * @brief Get current beacon status
 * @param status Output: status information
 */
void test_beacon_get_status(beacon_status_t* status);

/**
 * @brief Set beacon operating mode
 * @param mode Operating mode (responder or master)
 */
void test_beacon_set_mode(beacon_mode_e mode);

/**
 * @brief Set beacon counter value
 * @param value Counter value (0-65535)
 */
void test_beacon_set_counter(uint16_t value);

/**
 * @brief Send ping message to specified address
 * @param dest_addr Destination address (use 0xFFFF for broadcast)
 * @return true if sent successfully, false otherwise
 */
bool test_beacon_send_ping(uint16_t dest_addr);
