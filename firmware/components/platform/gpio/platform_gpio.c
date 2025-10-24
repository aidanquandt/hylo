#include "platform_gpio.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

// todo: add input for led to toggle - need to make some pinout abstraction
void platform_gpio_toggle_pin(void){
    BSP_LED_Toggle(LED_GREEN);
}