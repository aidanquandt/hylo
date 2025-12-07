/*---------------------------------------------------------------------------
 * @file    uart_cmd_router.c
 * @brief   UART command routing and dispatch implementation
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_cmd_router.h"
#include "error_handler.h"
#include "uart_manager.h"
#include <ctype.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_REGISTERED_MODULES 16U // Max number of module handlers
#define MAX_TOKEN_LENGTH 32U       // Max length for module/action/target
#define MAX_CMD_LENGTH 128U        // Max command length

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC cmd_module_entry_t registered_modules[MAX_REGISTERED_MODULES];
STATIC uint8_t num_registered = 0U;
STATIC bool router_initialized = false; // Guard against multiple init calls

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC cmd_action_e parse_action(const char* action_str);
STATIC const char* skip_whitespace(const char* str);
STATIC void uart_cmd_router_handle_builtin(const cmd_parsed_t* parsed);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Parse action string to enum
 */
STATIC cmd_action_e parse_action(const char* action_str)
{
    if (strcmp(action_str, "get") == 0)
    {
        return CMD_ACTION_GET;
    }
    else if (strcmp(action_str, "set") == 0)
    {
        return CMD_ACTION_SET;
    }
    else if (strcmp(action_str, "req") == 0)
    {
        return CMD_ACTION_REQ;
    }
    return CMD_ACTION_UNKNOWN;
}

/**
 * @brief Skip leading whitespace
 */
STATIC const char* skip_whitespace(const char* str)
{
    while (*str && isspace((unsigned char)*str))
    {
        str++;
    }
    return str;
}

/**
 * @brief Handle built-in commands (help, list)
 */
STATIC void uart_cmd_router_handle_builtin(const cmd_parsed_t* parsed)
{
    if (strcmp(parsed->module, "help") == 0)
    {
        uart_manager_print("\r\nCommands: <module>.<action> <target> [args]\r\n");
        uart_manager_print("Actions: get, set, req\r\n\r\n");
        uart_manager_print("Modules (%u):  ", num_registered);
        for (uint8_t i = 0; i < num_registered; i++)
        {
            uart_manager_print("%s%s", registered_modules[i].module_name,
                               (i < num_registered - 1) ? ", " : "\r\n");
        }
        uart_manager_print("Built-in: help, list\r\n");
    }
    else if (strcmp(parsed->module, "list") == 0)
    {
        uart_manager_print("\r\nModules (%u):\r\n", num_registered);
        for (uint8_t i = 0; i < num_registered; i++)
        {
            uart_manager_print("  %u: %s\r\n", i + 1, registered_modules[i].module_name);
        }
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void uart_cmd_router_init(void)
{
    // Guard against multiple initialization
    if (router_initialized)
    {
        return;
    }

    num_registered = 0U;
    memset(registered_modules, 0, sizeof(registered_modules));
    router_initialized = true;
}

bool uart_cmd_router_register(const char* module_name, cmd_handler_fn_t handler)
{
    if (module_name == NULL || handler == NULL)
    {
        return false;
    }

    if (num_registered >= MAX_REGISTERED_MODULES)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "uart_cmd", "Module registration full (%u/%u)",
                          num_registered, MAX_REGISTERED_MODULES);
        return false;
    }

    // Check for duplicate registration
    for (uint8_t i = 0; i < num_registered; i++)
    {
        if (strcmp(registered_modules[i].module_name, module_name) == 0)
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "uart_cmd", "Module '%s' already registered",
                              module_name);
            return false;
        }
    }

    registered_modules[num_registered].module_name = module_name;
    registered_modules[num_registered].handler = handler;
    num_registered++;

    return true;
}

bool uart_cmd_router_parse(const char* cmd_string, cmd_parsed_t* parsed)
{
    if (cmd_string == NULL || parsed == NULL)
    {
        return false;
    }

    // Clear output structure
    memset(parsed, 0, sizeof(cmd_parsed_t));

    // Use local buffer instead of static to avoid re-entrancy issues
    char local_parse_buffer[MAX_CMD_LENGTH];
    strncpy(local_parse_buffer, cmd_string, sizeof(local_parse_buffer) - 1);
    local_parse_buffer[sizeof(local_parse_buffer) - 1] = '\0';

    // Skip leading whitespace
    char* p = local_parse_buffer;
    while (*p && isspace((unsigned char)*p))
    {
        p++;
    }

    if (*p == '\0')
    {
        return false; // Empty command
    }

    // Parse module name (before '.')
    char* dot = strchr(p, '.');
    if (dot == NULL)
    {
        // No dot - treat entire command as module (for built-ins like "help")
        strncpy(parsed->module, p, sizeof(parsed->module) - 1);
        parsed->module[sizeof(parsed->module) - 1] = '\0';
        parsed->action = CMD_ACTION_UNKNOWN;
        return true;
    }

    // Extract module
    size_t module_len = dot - p;
    if (module_len >= sizeof(parsed->module))
    {
        module_len = sizeof(parsed->module) - 1;
    }
    strncpy(parsed->module, p, module_len);
    parsed->module[module_len] = '\0';

    // Move past dot
    p = dot + 1;

    // Parse action (get/set/req) - use local buffer
    char local_action_buf[MAX_TOKEN_LENGTH];
    char* space = strchr(p, ' ');
    if (space != NULL)
    {
        size_t action_len = space - p;
        if (action_len >= sizeof(local_action_buf))
        {
            action_len = sizeof(local_action_buf) - 1;
        }
        strncpy(local_action_buf, p, action_len);
        local_action_buf[action_len] = '\0';
        p = space + 1;
    }
    else
    {
        // No space - action is rest of string
        strncpy(local_action_buf, p, sizeof(local_action_buf) - 1);
        local_action_buf[sizeof(local_action_buf) - 1] = '\0';
        p = NULL;
    }

    parsed->action = parse_action(local_action_buf);

    // Parse target (next token)
    if (p != NULL)
    {
        p = (char*)skip_whitespace(p);
        space = strchr(p, ' ');

        if (space != NULL)
        {
            size_t target_len = space - p;
            if (target_len >= sizeof(parsed->target))
            {
                target_len = sizeof(parsed->target) - 1;
            }
            strncpy(parsed->target, p, target_len);
            parsed->target[target_len] = '\0';

            // Rest is arguments
            p = space + 1;
            p = (char*)skip_whitespace(p);
            strncpy(parsed->args, p, sizeof(parsed->args) - 1);
            parsed->args[sizeof(parsed->args) - 1] = '\0';
        }
        else
        {
            // No more tokens - this is the target
            strncpy(parsed->target, p, sizeof(parsed->target) - 1);
            parsed->target[sizeof(parsed->target) - 1] = '\0';
            // args already zeroed by memset
        }
    }

    return true;
}

void uart_cmd_router_dispatch(const char* cmd_string)
{
    cmd_parsed_t parsed;

    if (!uart_cmd_router_parse(cmd_string, &parsed))
    {
        uart_manager_print("ERR: Invalid format\r\n");
        return;
    }

    // Handle built-in commands
    if (strcmp(parsed.module, "help") == 0 || strcmp(parsed.module, "list") == 0)
    {
        uart_cmd_router_handle_builtin(&parsed);
        return;
    }

    // Check if action is valid (not UNKNOWN)
    if (parsed.action == CMD_ACTION_UNKNOWN)
    {
        uart_manager_print("ERR: Unknown action (use get/set/req)\r\n");
        return;
    }

    // Check if target is provided for commands that need it
    if (strlen(parsed.target) == 0)
    {
        uart_manager_print("ERR: Missing target\r\n");
        return;
    }

    // Find and call registered handler
    for (uint8_t i = 0; i < num_registered; i++)
    {
        if (strcmp(registered_modules[i].module_name, parsed.module) == 0)
        {
            bool handled = registered_modules[i].handler(&parsed);
            if (!handled)
            {
                const char* action_str = (parsed.action == CMD_ACTION_GET)   ? "get"
                                         : (parsed.action == CMD_ACTION_SET) ? "set"
                                                                             : "req";
                uart_manager_print("ERR: '%s' no cmd '%s %s'\r\n", parsed.module, action_str,
                                   parsed.target);
            }
            return;
        }
    }

    // No handler found
    uart_manager_print("ERR: Unknown module '%s'\r\n", parsed.module);
}
