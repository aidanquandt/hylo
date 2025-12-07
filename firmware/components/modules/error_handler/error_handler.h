#pragma once

/*---------------------------------------------------------------------------
 * @file    error_handler.h
 * @brief   Centralized error handling and logging module
 * @details Provides error logging with severity levels, error history tracking,
 *          and fatal error handling with LED indication. Thread-safe for use
 *          from any RTOS task context.
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define ERROR_MSG_MAX_LENGTH 80U

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

/**
 * @brief Error severity levels
 */
typedef enum
{
    ERROR_SEVERITY_INFO = 0U,    // Informational message
    ERROR_SEVERITY_WARNING = 1U, // Warning - potential issue
    ERROR_SEVERITY_ERROR = 2U,   // Error - functionality impaired
    ERROR_SEVERITY_FATAL = 3U    // Fatal - system cannot continue
} error_severity_e;

/**
 * @brief Error record structure
 */
typedef struct
{
    error_severity_e severity;
    char module[16];
    char message[ERROR_MSG_MAX_LENGTH];
    uint32_t timestamp_ms; // System uptime when error occurred
} error_record_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Log an error with severity level
 * @param severity Error severity level
 * @param module Module name (null-terminated, max 15 chars)
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @note Thread-safe: Can be called from any task context
 * @note For FATAL errors, use error_handler_fatal() instead
 */
void error_handler_log(error_severity_e severity, const char* module, const char* format, ...)
    __attribute__((format(printf, 3, 4)));

/**
 * @brief Report fatal error and halt system
 * @param module Module name (null-terminated, max 15 chars)
 * @param format Printf-style format string
 * @param ... Variable arguments
 * @note This function does not return - system will halt
 * @note Logs error, sets error LED pattern, and enters infinite loop
 * @note Provides visual indication via LED blinking (if available)
 */
void error_handler_fatal(const char* module, const char* format, ...)
    __attribute__((format(printf, 2, 3))) __attribute__((noreturn));

/**
 * @brief Get total count of errors by severity level
 * @param severity Severity level to query
 * @return Total count of errors at this severity level since init
 */
uint32_t error_handler_get_count(error_severity_e severity);

/**
 * @brief Get the most recent error record
 * @param record Pointer to error_record_t to fill
 * @return true if error retrieved, false if no errors in history
 */
bool error_handler_get_last_error(error_record_t* record);

/**
 * @brief Clear error history and counters
 * @note Does not reset fatal error state (system must be reset)
 */
void error_handler_clear_history(void);
