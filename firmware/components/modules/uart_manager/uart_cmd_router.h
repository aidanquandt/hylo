/*---------------------------------------------------------------------------
 * @file    uart_cmd_router.h
 * @brief   UART command router - parses commands and calls module APIs
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize command router
 * @note Must be called before dispatching commands
 * @note Idempotent - safe to call multiple times
 */
void uart_cmd_router_init(void);

/**
 * @brief Parse and route command to appropriate module
 * @param cmd_string Command string (e.g., "beacon.get.status" or "beacon.set.mode responder")
 *
 * @note Command format: <module>.<action>.<target> [args]
 * @note Called automatically by uart_manager when command received
 */
void uart_cmd_router_dispatch(const char* cmd_string);
