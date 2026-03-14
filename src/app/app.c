/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "common.h"
#include <stdio.h>
#include "datalogger.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "system_halt.h"
#include "protocol.pb.h"
#include "gpio_driver.h"
#include "os_driver.h"
#include "system_driver.h"
#include "task.h"
#include "uart_driver.h"
#include "uart_protocol_task.h"
#include "uwb.h"
#include "sdcard_driver.h"
#include "spi_driver.h"
#include "sensor_fusion.h"
#include "imu.h"
#include "uwb_node.h"
#include "ota_config.h"
#include "twr.h"
#include "twr_manager.h"
#include "wifi.h"
#include "watchdog.h"

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void app_post_module_initialization(void);

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void app_post_module_initialization(void)
{
#if FEATURE_AUTO_CONFIGURE_ADDRESS_FROM_UUID
    // Auto-configure UWB address based on device UUID if known
    system_driver_device_init();

    system_driver_device_info_t dev_info;
    if (system_driver_device_get_info(&dev_info))
    {
        if (dev_info.is_known_device)
        {
            uint16_t current_pan = uwb_get_pan_id();

            uwb_set_address(dev_info.assigned_address, current_pan);
        }
        else
        {
            AppDeviceNotInMappingTableEvent ev = AppDeviceNotInMappingTableEvent_init_zero;
            ev.uuid_word2 = dev_info.uuid_word2;
            protocol_tx_AppDeviceNotInMappingTableEvent(&ev);
            AppUsingDefaultAddressEvent ev2 = AppUsingDefaultAddressEvent_init_zero;
            ev2.address = uwb_get_address();
            protocol_tx_AppUsingDefaultAddressEvent(&ev2);
        }
    }
    else
    {
        AppFailedToInitDeviceIdEvent ev = AppFailedToInitDeviceIdEvent_init_zero;
        protocol_tx_AppFailedToInitDeviceIdEvent(&ev);
    }
#endif
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/

void app_init(void)
{
    uart_driver_init();
    spi_driver_init();
    uart_protocol_task_start();

    sensor_fusion_init();
    uwb_init();
    imu_init();
    uwb_node_init();
    ota_config_init();
    twr_init();
    twr_manager_init();
    wifi_init();
    watchdog_init(); // Must be last — monitors all other modules

    app_post_module_initialization();
    sdcard_driver_init();
    vTaskDelete(NULL); // Delete init task - scheduler continues with created tasks
}
