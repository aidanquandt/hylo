
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
    SENSOR_FUSION_MODULE = 0U,
    DATALOGGER_MODULE,
    NODE_MODULE,
    TDMA_MODULE,
    TWR_MODULE,
    DW3000_TEST_MODULE,
    BMI323_TEST_MODULE,
    NUM_MODULES
} modules_E;

typedef struct 
{
    void (*module_init)(void);
    void (*module_process_1Hz)(void);
    void (*module_process_10Hz)(void);
    void (*module_process_100Hz)(void);
    void (*module_process_1kHz)(void);
} module_S;

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S* const modules[NUM_MODULES];