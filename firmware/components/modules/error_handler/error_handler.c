/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "error_handler.h"
#include "common.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "feature_config.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "semphr.h"
#include "task.h"
#include "uart_manager.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ERROR_HISTORY_SIZE 16U

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void error_handler_add_to_history(const error_record_t* record);
STATIC const char* error_handler_severity_to_string(error_severity_e severity);
STATIC void error_handler_increment_counter(error_severity_e severity);
STATIC void error_handler_init(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
extern const module_S error_handler_module;
const module_S error_handler_module = {
    .module_name = "error",
    .module_init = error_handler_init,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC error_record_t error_history[ERROR_HISTORY_SIZE];
STATIC uint32_t error_history_head   = 0U;
STATIC uint32_t error_history_count  = 0U;
STATIC SemaphoreHandle_t error_mutex = NULL;

STATIC uint32_t error_count_info    = 0U;
STATIC uint32_t error_count_warning = 0U;
STATIC uint32_t error_count_error   = 0U;
STATIC uint32_t error_count_fatal   = 0U;
STATIC uint32_t dropped_error_count = 0U;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void error_handler_init(void)
{
    error_mutex = xSemaphoreCreateMutex();
    if (error_mutex == NULL)
    {
        for (;;)
            ;
    }

    memset(error_history, 0, sizeof(error_history));
    error_history_head  = 0U;
    error_history_count = 0U;

    error_count_info    = 0U;
    error_count_warning = 0U;
    error_count_error   = 0U;
    error_count_fatal   = 0U;
    dropped_error_count = 0U;
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

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void error_handler_log(error_severity_e severity, const char* module, const char* format, ...)
{
    if (module == NULL || format == NULL)
    {
        return;
    }

    // Check if we're in an ISR context
    bool from_isr = (xPortIsInsideInterrupt() == pdTRUE);

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
#if FEATURE_UART_LOGGING
        const char* severity_str = error_handler_severity_to_string(severity);
        uart_manager_print("[%s] %s: %s\r\n", severity_str, module, message);
#endif
        return;
    }

    // Normal task context - use mutex protection
    if (xSemaphoreTake(error_mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        if (dropped_error_count < UINT32_MAX)
        {
            dropped_error_count++;
        }
        return;
    }

    error_record_t record;
    record.severity     = severity;
    record.timestamp_ms = xTaskGetTickCount();

    strncpy(record.module, module, sizeof(record.module) - 1);
    record.module[sizeof(record.module) - 1] = '\0';

    va_list args;
    va_start(args, format);
    vsnprintf(record.message, sizeof(record.message), format, args);
    va_end(args);

    record.message[sizeof(record.message) - 1] = '\0';

    error_handler_add_to_history(&record);
    error_handler_increment_counter(severity);
    xSemaphoreGive(error_mutex);

#if FEATURE_UART_LOGGING
    const char* severity_str = error_handler_severity_to_string(severity);
    uart_manager_print("[%s] %s: %s\r\n", severity_str, record.module, record.message);
#endif
}

void error_handler_fatal(const char* module, const char* format, ...)
{
    char message[ERROR_MSG_MAX_LENGTH];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    message[sizeof(message) - 1] = '\0';

#if FEATURE_UART_LOGGING
    // Note: Fatal errors are printed regardless of logging setting in many systems
    // since they indicate critical failures. Change this behavior if needed.
    uart_manager_print("\r\n");
    uart_manager_print("==============================================\r\n");
    uart_manager_print("         FATAL ERROR - SYSTEM HALTED          \r\n");
    uart_manager_print("==============================================\r\n");
    uart_manager_print("Module:  %s\r\n", module);
    uart_manager_print("Message: %s\r\n", message);
    uart_manager_print("Time:    %lu ms\r\n", (unsigned long)xTaskGetTickCount());
    uart_manager_print("==============================================\r\n");
    uart_manager_print("System halted. Power cycle required.\r\n");
    uart_manager_print("\r\n");
#endif

    vTaskDelay(pdMS_TO_TICKS(100));

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

    if (xSemaphoreTake(error_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
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
        xSemaphoreGive(error_mutex);
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

    if (xSemaphoreTake(error_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        if (error_history_count > 0U)
        {
            uint32_t last_index =
                (error_history_head == 0U) ? (ERROR_HISTORY_SIZE - 1U) : (error_history_head - 1U);
            memcpy(record, &error_history[last_index], sizeof(error_record_t));
            success = true;
        }
        xSemaphoreGive(error_mutex);
    }

    return success;
}

void error_handler_clear_history(void)
{
    if (xSemaphoreTake(error_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        memset(error_history, 0, sizeof(error_history));
        error_history_head  = 0U;
        error_history_count = 0U;

        error_count_info    = 0U;
        error_count_warning = 0U;
        error_count_error   = 0U;
        error_count_fatal   = 0U;
        dropped_error_count = 0U;

        xSemaphoreGive(error_mutex);
    }
}