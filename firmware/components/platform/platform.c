#include "platform/platform.h"
#include "stm32h7xx_hal.h"

uint32_t Platform_Ticks(void) {
    return HAL_GetTick();
}
