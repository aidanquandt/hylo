
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UART_MANAGER_MODULE = 0U,
    ERROR_HANDLER_MODULE,
    DATALOGGER_MODULE,
    SENSOR_FUSION_MODULE,
    UWB_MODULE,
    IMU_MODULE,
    UWB_NODE_MODULE,
    OTA_CONFIG_MODULE,
    TWR_MODULE,
    TWR_MANAGER_MODULE,
    WIFI_MODULE,
    WATCHDOG_MODULE, // Must be last - runs after all other modules
    NUM_MODULES
} modules_E;

/**
 * Module interface
 *
 * Each module provides:
 * - module_name: Human-readable name for logging/debugging
 * - module_init: Pre-RTOS initialization (hardware setup, default state)
 * - module_create_tasks: Post-RTOS task creation (NULL if no tasks needed)
 *
 * Modules create their own FreeRTOS tasks as needed. Task priorities are
 * defined in common/task_config.h for system-wide visibility.
 */
typedef struct
{
    const char* module_name;
    void (*module_init)(void);
    void (*module_create_tasks)(void);
} module_S;

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S* const modules[NUM_MODULES];