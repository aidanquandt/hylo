/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_timer.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define PLATFORM_TIMER_NS_PER_TICK (101.81818182f)
#define PLATFORM_TIMESTAMP_TIMER htim5

/*---------------------------------------------------------------------------
 * External Variables
 *---------------------------------------------------------------------------*/
extern TIM_HandleTypeDef PLATFORM_TIMESTAMP_TIMER;

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

uint32_t platform_get_timestamp(void)
{
    return __HAL_TIM_GET_COUNTER(&PLATFORM_TIMESTAMP_TIMER);
}

uint32_t platform_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp)
{
    uint32_t elapsed_ticks = end_timestamp - start_timestamp;
    return (uint32_t)((float)elapsed_ticks * (PLATFORM_TIMER_NS_PER_TICK / 1000.0f));
}