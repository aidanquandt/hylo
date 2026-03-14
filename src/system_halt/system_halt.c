/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "system_halt.h"
#include "FreeRTOS.h"
#include "gpio_driver.h"
#include "os_driver.h"
#include "protocol_tx.h"
#include "task.h"
#include "protocol.pb.h"
#include <string.h>

void system_halt(const char* module, const char* message)
{
    if (module != NULL && message != NULL)
    {
        SystemFatalEvent ev = SystemFatalEvent_init_zero;
        strncpy(ev.module, module, sizeof(ev.module) - 1);
        ev.module[sizeof(ev.module) - 1] = '\0';
        strncpy(ev.message, message, sizeof(ev.message) - 1);
        ev.message[sizeof(ev.message) - 1] = '\0';
        protocol_tx_SystemFatalEvent(&ev);
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    os_driver_critical_enter();

    uint32_t counter = 0U;
    for (;;)
    {
        for (volatile uint32_t i = 0; i < 500000U; i++)
            ;

        counter++;
        if (counter >= 5U)
        {
            gpio_driver_toggle_led_green();
            counter = 0U;
        }
    }
}
