/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "module.h"
#include "uart_manager.h"
#include "datalogger.h"
#include "node.h"
#include "sensor_fusion.h"
#include "tdma.h"
#include "twr.h"
#include "uwb_test.h"
#include "imu_test.h"

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S uart_manager_module;
extern const module_S sensor_fusion_module;
extern const module_S datalogger_module;
extern const module_S node_module;
extern const module_S tdma_module;
extern const module_S twr_module;
extern const module_S uwb_test_module;
extern const module_S imu_test_module;

const module_S* const modules[NUM_MODULES] = {
    [UART_MANAGER_MODULE]  = &uart_manager_module,  // First! (other modules may print during init)
    [SENSOR_FUSION_MODULE] = &sensor_fusion_module,
    [DATALOGGER_MODULE]    = &datalogger_module,
    [NODE_MODULE]          = &node_module,
    [TDMA_MODULE]          = &tdma_module,
    [TWR_MODULE]           = &twr_module,
    [UWB_TEST_MODULE]      = &uwb_test_module,
    [IMU_TEST_MODULE]      = &imu_test_module,
};