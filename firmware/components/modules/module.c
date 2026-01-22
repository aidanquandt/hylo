/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "module.h"
#include "datalogger.h"
#include "error_handler.h"
#include "imu.h"
#include "ranging/ranging.h"
#include "sensor_fusion.h"
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
extern const module_S ranging_module;

const module_S* const modules[NUM_MODULES] = {
    [UART_MANAGER_MODULE] = &uart_manager_module,
    [ERROR_HANDLER_MODULE] = &error_handler_module,
    [SENSOR_FUSION_MODULE] = &sensor_fusion_module,
    [DATALOGGER_MODULE] = &datalogger_module,
    [UWB_MODULE] = &uwb_module,
    [IMU_MODULE] = &imu_module,
    [RANGING_MODULE] = &ranging_module,
};