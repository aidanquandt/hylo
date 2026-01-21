/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "error_handler.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "stm32h7xx.h"
#include "uart_manager.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ERROR_HISTORY_SIZE 16U

/*---------------------------------------------------------------------------
 * Module Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void error_handler_init(void);
STATIC void error_handler_process_10Hz(void);

extern const module_S error_handler_module;
const module_S error_handler_module = {
    .module_name = "error",
    .module_init = error_handler_init,
    .module_process_10Hz = error_handler_process_10Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC error_record_t error_history[ERROR_HISTORY_SIZE];
STATIC uint32_t error_history_head = 0U;
STATIC uint32_t error_history_count = 0U;
STATIC platform_os_mutex_t error_mutex = NULL;

STATIC uint32_t error_count_info = 0U;
STATIC uint32_t error_count_warning = 0U;
STATIC uint32_t error_count_error = 0U;
STATIC uint32_t error_count_fatal = 0U;
STATIC uint32_t dropped_error_count = 0U;

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
    error_mutex = platform_os_mutex_create();
    if (error_mutex == NULL)
    {
        for (;;)
            ;
    }

    memset(error_history, 0, sizeof(error_history));
    error_history_head = 0U;
    error_history_count = 0U;

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
 * Public Function Implementations (Accessor Functions for Router)
 *---------------------------------------------------------------------------*/

void error_handler_log(error_severity_e severity, const char* module, const char* format, ...)
{
    if (module == NULL || format == NULL)
    {
        return;
    }

    // Check if we're in an ISR context
    bool from_isr = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U;

    // If in ISR, skip mutex and history to avoid FreeRTOS API violations
    if (from_isr)
    {
        // Format message directly for printing
        char message[ERROR_MSG_MAX_LENGTH];
        va_list args;
        va_start(args, format);
        vsnprintf(message, sizeof(message), format, args);
        va_end(args);
        message[sizeof(message) - 1] = '\0';

        // Increment dropped count since we can't safely update history
        if (dropped_error_count < UINT32_MAX)
        {
            dropped_error_count++;
        }

        // Print directly (uart_manager_print handles ISR context)
        const char* severity_str = error_handler_severity_to_string(severity);
        uart_manager_print("[%s] %s: %s\r\n", severity_str, module, message);
        return;
    }

    // Normal task context - use mutex protection
    if (!platform_os_mutex_take(error_mutex, 100))
    {
        if (dropped_error_count < UINT32_MAX)
        {
            dropped_error_count++;
        }
        return;
    }

    error_record_t record;
    record.severity = severity;
    record.timestamp_ms = platform_os_gettick();

    strncpy(record.module, module, sizeof(record.module) - 1);
    record.module[sizeof(record.module) - 1] = '\0';

    va_list args;
    va_start(args, format);
    vsnprintf(record.message, sizeof(record.message), format, args);
    va_end(args);

    record.message[sizeof(record.message) - 1] = '\0';

    error_handler_add_to_history(&record);
    error_handler_increment_counter(severity);
    platform_os_mutex_give(error_mutex);

    const char* severity_str = error_handler_severity_to_string(severity);
    uart_manager_print("[%s] %s: %s\r\n", severity_str, record.module, record.message);
}

void error_handler_fatal(const char* module, const char* format, ...)
{
    char message[ERROR_MSG_MAX_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    message[sizeof(message) - 1] = '\0';

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

    platform_os_delay_ms(100);

    fatal_error_occurred = true;
    platform_os_critical_enter();

    uint32_t counter = 0U;
    for (;;)
    {
        for (volatile uint32_t i = 0; i < 500000U; i++)
            ;

        counter++;
        if (counter >= 5U)
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

    memcpy(&error_history[error_history_head], record, sizeof(error_record_t));

    error_history_head = (error_history_head + 1U) % ERROR_HISTORY_SIZE;

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
