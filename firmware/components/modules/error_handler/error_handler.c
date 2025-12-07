/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "error_handler.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "uart_cmd_router.h"
#include "uart_manager.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ERROR_HISTORY_SIZE 16U // Circular buffer size for error history

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void error_handler_init(void);
STATIC void error_handler_process_10Hz(void);
STATIC bool error_handler_cmd_handler(const cmd_parsed_t* parsed);

extern const module_S error_handler_module;
const module_S error_handler_module = {
    .module_name = "error",
    .module_init = error_handler_init,
    .module_process_10Hz = error_handler_process_10Hz,
    .module_cmd_handler = error_handler_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC error_record_t error_history[ERROR_HISTORY_SIZE];
STATIC uint32_t error_history_head = 0U;       // Next write position
STATIC uint32_t error_history_count = 0U;      // Total entries (saturates at buffer size)
STATIC platform_os_mutex_t error_mutex = NULL; // Mutex for thread-safe access

// Error counters by severity
STATIC uint32_t error_count_info = 0U;
STATIC uint32_t error_count_warning = 0U;
STATIC uint32_t error_count_error = 0U;
STATIC uint32_t error_count_fatal = 0U;
STATIC uint32_t dropped_error_count = 0U; // Errors dropped due to mutex timeout

// Fatal error state
STATIC bool fatal_error_occurred = false;
STATIC uint32_t fatal_led_blink_counter = 0U;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void error_handler_add_to_history(const error_record_t* record);
STATIC const char* error_handler_severity_to_string(error_severity_e severity);
STATIC void error_handler_increment_counter(error_severity_e severity);

/*---------------------------------------------------------------------------
 * Module Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void error_handler_init(void)
{
    // Create mutex for thread-safe access
    error_mutex = platform_os_mutex_create();
    if (error_mutex == NULL)
    {
        // Can't use error_handler_fatal here (would recurse), just halt
        for (;;)
            ;
    }

    // Clear error history
    memset(error_history, 0, sizeof(error_history));
    error_history_head = 0U;
    error_history_count = 0U;

    // Reset counters
    error_count_info = 0U;
    error_count_warning = 0U;
    error_count_error = 0U;
    error_count_fatal = 0U;
    dropped_error_count = 0U;

    fatal_error_occurred = false;
    fatal_led_blink_counter = 0U;
}

STATIC void error_handler_process_10Hz(void)
{
    // If fatal error occurred, blink LED at 1Hz (5 ticks on, 5 ticks off)
    if (fatal_error_occurred)
    {
        fatal_led_blink_counter++;
        if (fatal_led_blink_counter >= 5U)
        {
            platform_gpio_toggle_led_green();
            fatal_led_blink_counter = 0U;
        }
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void error_handler_log(error_severity_e severity, const char* module, const char* format, ...)
{
    if (module == NULL || format == NULL)
    {
        return;
    }

    // Take mutex for thread-safe operation
    if (!platform_os_mutex_take(error_mutex, 100))
    {
        // Mutex timeout - track dropped error (lock-free increment is safe here)
        if (dropped_error_count < UINT32_MAX)
        {
            dropped_error_count++;
        }
        return;
    }

    // Create error record
    error_record_t record;
    record.severity = severity;
    record.timestamp_ms = platform_os_gettick();

    // Copy module name (truncate if too long)
    strncpy(record.module, module, sizeof(record.module) - 1);
    record.module[sizeof(record.module) - 1] = '\0';

    // Format message
    va_list args;
    va_start(args, format);
    vsnprintf(record.message, sizeof(record.message), format, args);
    va_end(args);

    // Ensure null termination
    record.message[sizeof(record.message) - 1] = '\0';

    // Add to history
    error_handler_add_to_history(&record);

    // Increment counter
    error_handler_increment_counter(severity);

    // Release mutex
    platform_os_mutex_give(error_mutex);

    // Log to UART (outside mutex to avoid blocking)
    const char* severity_str = error_handler_severity_to_string(severity);
    uart_manager_print("[%s] %s: %s\r\n", severity_str, record.module, record.message);
}

void error_handler_fatal(const char* module, const char* format, ...)
{
    // Log fatal error
    char message[ERROR_MSG_MAX_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    message[sizeof(message) - 1] = '\0';

    // Print to UART immediately
    uart_manager_print("\r\n");
    uart_manager_print("==============================================\r\n");
    uart_manager_print("         FATAL ERROR - SYSTEM HALTED          \r\n");
    uart_manager_print("==============================================\r\n");
    uart_manager_print("Module:  %s\r\n", module);
    uart_manager_print("Message: %s\r\n", message);
    uart_manager_print("Time:    %lu ms\r\n", (unsigned long)platform_os_gettick());
    uart_manager_print("==============================================\r\n");
    uart_manager_print("System halted. Power cycle required.\r\n");
    uart_manager_print("\r\n");

    // Give UART time to flush
    platform_os_delay_ms(100);

    // Set fatal error flag for LED blinking (if scheduler running)
    fatal_error_occurred = true;

    // Disable interrupts to prevent further task switching
    platform_os_critical_enter();

    // Halt with LED blinking pattern (blocking loop)
    uint32_t counter = 0U;
    for (;;)
    {
        // Simple delay loop (non-RTOS)
        for (volatile uint32_t i = 0; i < 500000U; i++)
            ; // ~100ms at typical CPU speed

        counter++;
        if (counter >= 5U) // Toggle every ~500ms (1Hz blink)
        {
            platform_gpio_toggle_led_green();
            counter = 0U;
        }
    }
}

uint32_t error_handler_get_count(error_severity_e severity)
{
    uint32_t count = 0U;

    if (platform_os_mutex_take(error_mutex, 100))
    {
        switch (severity)
        {
            case ERROR_SEVERITY_INFO:
                count = error_count_info;
                break;
            case ERROR_SEVERITY_WARNING:
                count = error_count_warning;
                break;
            case ERROR_SEVERITY_ERROR:
                count = error_count_error;
                break;
            case ERROR_SEVERITY_FATAL:
                count = error_count_fatal;
                break;
            default:
                break;
        }
        platform_os_mutex_give(error_mutex);
    }

    return count;
}

bool error_handler_get_last_error(error_record_t* record)
{
    if (record == NULL)
    {
        return false;
    }

    bool success = false;

    if (platform_os_mutex_take(error_mutex, 100))
    {
        if (error_history_count > 0U)
        {
            // Get most recent error (previous write position)
            uint32_t last_index =
                (error_history_head == 0U) ? (ERROR_HISTORY_SIZE - 1U) : (error_history_head - 1U);
            memcpy(record, &error_history[last_index], sizeof(error_record_t));
            success = true;
        }
        platform_os_mutex_give(error_mutex);
    }

    return success;
}

void error_handler_clear_history(void)
{
    if (platform_os_mutex_take(error_mutex, 100))
    {
        memset(error_history, 0, sizeof(error_history));
        error_history_head = 0U;
        error_history_count = 0U;

        error_count_info = 0U;
        error_count_warning = 0U;
        error_count_error = 0U;
        error_count_fatal = 0U;
        dropped_error_count = 0U;

        platform_os_mutex_give(error_mutex);
    }
}

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void error_handler_add_to_history(const error_record_t* record)
{
    if (record == NULL)
    {
        return;
    }

    // Add to circular buffer
    memcpy(&error_history[error_history_head], record, sizeof(error_record_t));

    // Update head pointer (wrap around)
    error_history_head = (error_history_head + 1U) % ERROR_HISTORY_SIZE;

    // Update count (saturate at buffer size)
    if (error_history_count < ERROR_HISTORY_SIZE)
    {
        error_history_count++;
    }
}

STATIC const char* error_handler_severity_to_string(error_severity_e severity)
{
    switch (severity)
    {
        case ERROR_SEVERITY_INFO:
            return "INFO";
        case ERROR_SEVERITY_WARNING:
            return "WARN";
        case ERROR_SEVERITY_ERROR:
            return "ERROR";
        case ERROR_SEVERITY_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

STATIC void error_handler_increment_counter(error_severity_e severity)
{
    switch (severity)
    {
        case ERROR_SEVERITY_INFO:
            if (error_count_info < UINT32_MAX)
            {
                error_count_info++;
            }
            break;
        case ERROR_SEVERITY_WARNING:
            if (error_count_warning < UINT32_MAX)
            {
                error_count_warning++;
            }
            break;
        case ERROR_SEVERITY_ERROR:
            if (error_count_error < UINT32_MAX)
            {
                error_count_error++;
            }
            break;
        case ERROR_SEVERITY_FATAL:
            if (error_count_fatal < UINT32_MAX)
            {
                error_count_fatal++;
            }
            break;
        default:
            break;
    }
}

/*---------------------------------------------------------------------------
 * Command Handler
 *---------------------------------------------------------------------------*/

STATIC bool error_handler_cmd_handler(const cmd_parsed_t* parsed)
{
    switch (parsed->action)
    {
        case CMD_ACTION_GET:
            if (strcmp(parsed->target, "status") == 0)
            {
                // Print error statistics
                uart_manager_print("\r\nError Handler Status:\r\n");
                uart_manager_print("%-12s %lu\r\n", "INFO:", (unsigned long)error_count_info);
                uart_manager_print("%-12s %lu\r\n", "WARNING:", (unsigned long)error_count_warning);
                uart_manager_print("%-12s %lu\r\n", "ERROR:", (unsigned long)error_count_error);
                uart_manager_print("%-12s %lu\r\n", "FATAL:", (unsigned long)error_count_fatal);
                uart_manager_print("%-12s %lu\r\n", "History:", (unsigned long)error_history_count);
                uart_manager_print("%-12s %lu\r\n", "Dropped:", (unsigned long)dropped_error_count);
                uart_manager_print("\r\n");
                return true;
            }
            else if (strcmp(parsed->target, "history") == 0)
            {
                // Print error history
                uart_manager_print("\r\nError History (most recent first):\r\n");
                uart_manager_print(
                    "-----------------------------------------------------------\r\n");

                if (error_history_count == 0U)
                {
                    uart_manager_print("No errors recorded.\r\n");
                }
                else
                {
                    // Print from most recent to oldest
                    uint32_t num_to_print = (error_history_count < ERROR_HISTORY_SIZE)
                                                ? error_history_count
                                                : ERROR_HISTORY_SIZE;
                    for (uint32_t i = 0; i < num_to_print; i++)
                    {
                        uint32_t index =
                            (error_history_head + ERROR_HISTORY_SIZE - 1U - i) % ERROR_HISTORY_SIZE;
                        const error_record_t* record = &error_history[index];

                        const char* severity_str =
                            error_handler_severity_to_string(record->severity);
                        uart_manager_print("[%6lu ms] %-5s | %-8s | %s\r\n",
                                           (unsigned long)record->timestamp_ms, severity_str,
                                           record->module, record->message);
                    }
                }

                uart_manager_print(
                    "-----------------------------------------------------------\r\n\r\n");
                return true;
            }
            break;

        case CMD_ACTION_SET:
            if (strcmp(parsed->target, "clear") == 0)
            {
                error_handler_clear_history();
                uart_manager_print("Error history cleared.\r\n");
                return true;
            }
            break;

        case CMD_ACTION_REQ:
        case CMD_ACTION_UNKNOWN:
        default:
            break;
    }

    return false;
}
