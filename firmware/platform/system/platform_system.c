/*---------------------------------------------------------------------------
 * @file    platform_system.c
 * @brief   Platform system utilities implementation
 *---------------------------------------------------------------------------*/

#include "platform_system.h"
#include "stm32h7xx.h" // Device header for NVIC_SystemReset

void platform_system_reset(void)
{
    NVIC_SystemReset();
}
