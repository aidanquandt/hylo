
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
    DATALOGGER_MODULE = 0U,
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

typedef struct
{
    const char* module_name;
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