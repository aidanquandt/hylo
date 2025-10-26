/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_timer.h"
#include "main.h"
#include "freertos.h"
#include "FreeRTOSConfig.h"

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void platform_configure_timer_for_run_time_stats(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long platform_get_run_time_counter_value(void)
{
    return DWT->CYCCNT;
}