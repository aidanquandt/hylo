/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "module.h"
#include "datalogger.h"
#include "error_handler.h"
#include "imu.h"
#include "node.h"
#include "sensor_fusion.h"
#include "twr/twr.h"
#include "twr_manager/twr_manager.h"
#include "uart_manager.h"
#include "uwb.h"

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S uart_manager_module;
extern const module_S error_handler_module;
extern const module_S sensor_fusion_module;
extern const module_S datalogger_module;
extern const module_S uwb_module;
extern const module_S imu_module;
extern const module_S node_module;
extern const module_S twr_module;
extern const module_S twr_manager_module;

const module_S* const modules[NUM_MODULES] = {
    [UART_MANAGER_MODULE]  = &uart_manager_module,
    [ERROR_HANDLER_MODULE] = &error_handler_module,
    [SENSOR_FUSION_MODULE] = &sensor_fusion_module,
    [DATALOGGER_MODULE]    = &datalogger_module,
    [UWB_MODULE]           = &uwb_module,
    [IMU_MODULE]           = &imu_module,
    [NODE_MODULE]          = &node_module,
    [TWR_MODULE]           = &twr_module,
    [TWR_MANAGER_MODULE]   = &twr_manager_module,
};