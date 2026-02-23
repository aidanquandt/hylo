/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "feature_config.h"
#include "main.h"
#include "module.h"
#include "platform_gpio.h"
#include "platform_system.h"
#include "task.h"
#include "uart_cmd_router.h"
#include "uwb.h"

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void app_initialize_modules(void);
STATIC void app_create_module_tasks(void);
STATIC void app_post_module_initialization(void);

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void app_initialize_modules(void)
{
    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++)
    {
        if (modules[module_idx]->module_init != NULL)
        {
            modules[module_idx]->module_init();
        }
    }
}

STATIC void app_create_module_tasks(void)
{
    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++)
    {
        if (modules[module_idx]->module_create_tasks != NULL)
        {
            modules[module_idx]->module_create_tasks();
        }
    }
}

STATIC void app_post_module_initialization(void)
{
    uart_cmd_router_init();

#if FEATURE_AUTO_CONFIGURE_ADDRESS_FROM_UUID
    // Auto-configure UWB address based on device UUID if known
    platform_system_device_init();

    platform_system_device_info_t dev_info;
    if (platform_system_device_get_info(&dev_info))
    {
        if (dev_info.is_known_device)
        {
            uint16_t current_pan = uwb_get_pan_id();

            uwb_set_address(dev_info.assigned_address, current_pan);
        }
        else
        {
            error_handler_log(ERROR_SEVERITY_WARNING, "app",
                              "Device not in mapping table (UUID word2: 0x%08lX)",
                              (unsigned long)dev_info.uuid_word2);
            error_handler_log(
                ERROR_SEVERITY_INFO, "app",
                "Using default address 0x%04X. Add to config/device_mapping.c if needed",
                uwb_get_address());
        }
    }
    else
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "app", "Failed to initialize device ID system");
    }
#endif
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void app_init(void)
{
    app_initialize_modules();
    app_create_module_tasks();
    app_post_module_initialization();
    vTaskDelete(NULL); // Delete init task - scheduler continues with created tasks
}