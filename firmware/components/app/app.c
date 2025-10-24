#include "app.h"
#include "common.h"
#include "main.h"
#include "platform.h"
#include "platform_gpio.h"

void App_Init(void) {
    // Put application one-time initialization here (after HAL and BSP init)
}

TCM_FUNCTION void App_Loop(void) {
    // Simple heartbeat: toggle LED and delay
    platform_gpio_toggle_pin();
    platform_delay_ms(100U);
}
