#pragma once
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
typedef enum
{
    ERROR_SEVERITY_INFO = 0U,    // Informational message
    ERROR_SEVERITY_WARNING = 1U, // Warning - potential issue
    ERROR_SEVERITY_ERROR = 2U,   // Error - functionality impaired
    ERROR_SEVERITY_FATAL = 3U    // Fatal - system cannot continue
} error_severity_e;

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

void error_handler_log(error_severity_e severity, const char* module, const char* format, ...)
    __attribute__((format(printf, 3, 4)));
void error_handler_fatal(const char* module, const char* format, ...)
    __attribute__((format(printf, 2, 3))) __attribute__((noreturn));
uint32_t error_handler_get_count(error_severity_e severity);
bool error_handler_get_last_error(error_record_t* record);
void error_handler_clear_history(void);
