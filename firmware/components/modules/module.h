
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
typedef struct cmd_parsed_s cmd_parsed_t;
typedef bool (*cmd_handler_fn_t)(const cmd_parsed_t* parsed);

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UART_MANAGER_MODULE = 0U, // Must initialize first (other modules may use UART)
    ERROR_HANDLER_MODULE,     // Initialize early (other modules may log errors)
    SENSOR_FUSION_MODULE,
    DATALOGGER_MODULE,
    NODE_MODULE,
    TDMA_MODULE,
    TWR_MODULE,
    UWB_MODULE,
    IMU_MODULE,
    NUM_MODULES
} modules_E;

/**
 * Module interface for two-phase initialization:
 * 1. module_init: Create RTOS resources (queues, semaphores) before scheduler starts
 * 2. module_create_task: Spawn tasks after scheduler is running
 * 3. module_process_*: Periodic callbacks driven by main application loop
 * 4. module_cmd_*: Optional UART command handler (auto-registered if non-NULL)
 */
typedef struct
{
    const char* module_name;
    void (*module_init)(void);
    void (*module_create_task)(void);
    void (*module_process_1Hz)(void);
    void (*module_process_10Hz)(void);
    void (*module_process_100Hz)(void);
    void (*module_process_1kHz)(void);

    // Optional: UART command interface
    cmd_handler_fn_t module_cmd_handler; // Command handler function or NULL
} module_S;

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S* const modules[NUM_MODULES];