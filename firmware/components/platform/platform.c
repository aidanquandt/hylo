#include "platform.h"
#include "stm32h7xx_hal.h"

uint32_t platform_gettick(void)
{
    return HAL_GetTick();
}

void platform_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
