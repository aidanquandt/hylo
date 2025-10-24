#include "app/app.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

void App_Init(void) {
    // Put application one-time initialization here (after HAL and BSP init)
}

void App_Loop(void) {
    // Simple heartbeat: toggle LED and delay
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(500);
}
