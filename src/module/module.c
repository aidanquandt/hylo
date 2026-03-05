/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "module.h"
#include "datalogger.h"
#include "error_handler.h"
#include "imu.h"
#include "ota_config.h"
#include "sensor_fusion.h"
#include "twr.h"
#include "twr_manager.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_node.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------
 * Public variables
 *---------------------------------------------------------------------------*/
extern const module_S uart_manager_module;
extern const module_S error_handler_module;
extern const module_S sensor_fusion_module;
extern const module_S datalogger_module;
extern const module_S watchdog_module;
extern const module_S uwb_module;
extern const module_S imu_module;
extern const module_S uwb_node_module;
extern const module_S ota_config_module;
extern const module_S twr_module;
extern const module_S twr_manager_module;

const module_S* const modules[NUM_MODULES] = {
    [UART_MANAGER_MODULE]  = &uart_manager_module,
    [ERROR_HANDLER_MODULE] = &error_handler_module,
    [SENSOR_FUSION_MODULE] = &sensor_fusion_module,
    [DATALOGGER_MODULE]    = &datalogger_module,
    [UWB_MODULE]           = &uwb_module,
    [IMU_MODULE]           = &imu_module,
    [UWB_NODE_MODULE]      = &uwb_node_module,
    [OTA_CONFIG_MODULE]    = &ota_config_module,
    [TWR_MODULE]           = &twr_module,
    [TWR_MANAGER_MODULE]   = &twr_manager_module,
    [WATCHDOG_MODULE]      = &watchdog_module, // Last - monitors all others
};