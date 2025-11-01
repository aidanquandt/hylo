/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "module.h"
#include "datalogger.h"
#include "node.h"
#include "sensor_fusion.h"
#include "tdma.h"
#include "twr.h"
#include "dw3000_test.h"

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S sensor_fusion_module;
extern const module_S datalogger_module;
extern const module_S node_module;
extern const module_S tdma_module;
extern const module_S twr_module;
extern const module_S dw3000_test_module;

const module_S* const modules[NUM_MODULES] = {
    [SENSOR_FUSION_MODULE] = &sensor_fusion_module,
    [DATALOGGER_MODULE]    = &datalogger_module,
    [NODE_MODULE]          = &node_module,
    [TDMA_MODULE]          = &tdma_module,
    [TWR_MODULE]           = &twr_module,
    [DW3000_TEST_MODULE]   = &dw3000_test_module,
};