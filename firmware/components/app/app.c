#include "app.h"
#include "common.h"
#include "main.h"
#include "platform.h"
#include "platform_gpio.h"

void app_init(void) {
    // Put application one-time initialization here (after HAL and BSP init)
}

void app_loop(void) {
    platform_gpio_toggle_pin();
    platform_delay_ms(100U);
}
