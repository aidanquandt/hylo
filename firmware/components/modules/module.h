
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
    UART_MANAGER_MODULE = 0U,  // Must initialize first (other modules may use UART)
    SENSOR_FUSION_MODULE,
    DATALOGGER_MODULE,
    NODE_MODULE,
    TDMA_MODULE,
    TWR_MODULE,
    DW3000_TEST_MODULE,
    IMU_TEST_MODULE,
    NUM_MODULES
} modules_E;

/**
 * Module interface for two-phase initialization:
 * 1. module_init: Create RTOS resources (queues, semaphores) before scheduler starts
 * 2. module_create_task: Spawn tasks after scheduler is running
 * 3. module_process_*: Periodic callbacks driven by main application loop
 */
typedef struct 
{
    void (*module_init)(void);
    void (*module_create_task)(void);
    void (*module_process_1Hz)(void);
    void (*module_process_10Hz)(void);
    void (*module_process_100Hz)(void);
    void (*module_process_1kHz)(void);
} module_S;

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S* const modules[NUM_MODULES];