/*---------------------------------------------------------------------------
 * @file    uart_cmd_router.h
 * @brief   UART command routing and dispatch system
 * @details Provides command parsing, routing to modules, and response handling
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

/**
 * @brief Command action types
 */
typedef enum
{
    CMD_ACTION_GET = 0, ///< Read/query data
    CMD_ACTION_SET,     ///< Write/configure data
    CMD_ACTION_REQ,     ///< Request action/operation
    CMD_ACTION_UNKNOWN  ///< Invalid/unrecognized action
} cmd_action_e;

/**
 * @brief Parsed command structure
 * @note Uses embedded buffers (not pointers) to avoid lifetime issues
 * @note Thread-safe: Safe to use in callbacks as data is copied
 */
typedef struct cmd_parsed_s
{
    char module[32];     ///< Module name (e.g., "imu", "uwb", "system")
    cmd_action_e action; ///< Action type (get, set, req)
    char target[32];     ///< Target parameter/resource (e.g., "temp", "channel")
    char args[128];      ///< Additional arguments (e.g., "5", "enabled")
} cmd_parsed_t;

/**
 * @brief Command handler function type
 * @param parsed Parsed command structure
 * @return true if command handled successfully, false if invalid/not recognized
 *
 * @note Handler should use uart_manager_print() to send responses
 * @note Keep handlers short to avoid blocking command reception
 * @note Handlers are called from 10Hz periodic context
 */
typedef bool (*cmd_handler_fn_t)(const cmd_parsed_t* parsed);

/**
 * @brief Command registration entry
 */
typedef struct
{
    const char* module_name;  // Module name to match
    cmd_handler_fn_t handler; // Handler function
} cmd_module_entry_t;

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize command router
 * @note Must be called before registering any modules
 * @note Idempotent - safe to call multiple times
 */
void uart_cmd_router_init(void);

/**
 * @brief Register module command handler
 * @param module_name Module name (e.g., "imu", "uwb", "system")
 * @param handler Handler function to call for this module
 * @return true if registered, false if table full or duplicate
 * @note Logs error message if registration fails
 */
bool uart_cmd_router_register(const char* module_name, cmd_handler_fn_t handler);

/**
 * @brief Route command to appropriate handler
 * @param cmd_string Full command string (e.g., "imu.get temp")
 *
 * @note This is called automatically by uart_manager callback
 * @note Handles built-in commands (help, list) internally
 */
void uart_cmd_router_dispatch(const char* cmd_string);

/**
 * @brief Parse command string into structured format
 * @param cmd_string Command string to parse
 * @param parsed Output parsed command structure
 * @return true if parsed successfully, false if invalid format
 *
 * @note Command format: <module>.<action> <target> [args]
 * @note Examples: "imu.get temp", "uwb.set channel 5"
 * @note Internal use - exposed for testing purposes
 */
bool uart_cmd_router_parse(const char* cmd_string, cmd_parsed_t* parsed);
