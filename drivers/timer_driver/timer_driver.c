/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "timer_driver.h"
#include "cmsis_os.h"
#include "main.h"
#include "tim.h"

#define TIMER_DRIVER_NS_PER_TICK (101.81818182f)

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;

static volatile uint32_t s_overflow_count;

uint32_t timer_driver_get_timestamp(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

uint32_t timer_driver_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp)
{
    uint32_t elapsed_ticks = end_timestamp - start_timestamp;
    return (uint32_t)((float)elapsed_ticks * (TIMER_DRIVER_NS_PER_TICK / 1000.0f));
}

void timer_driver_on_overflow(void)
{
    s_overflow_count++;
}

uint64_t timer_driver_get_time_us(void)
{
    uint32_t o1 = s_overflow_count;
    uint32_t c  = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t o2 = s_overflow_count;
    if (o1 != o2)
        c = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    return ((uint64_t)o2 << 32) | c;
}

uint32_t timer_driver_get_time_ms(void)
{
    return (uint32_t)(timer_driver_get_time_us() / 1000ULL);
}

/* Override HAL weak callback: TIM6 = HAL_IncTick(), TIM2 = 64-bit time overflow */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
// {
//     if (htim->Instance == TIM6)
//     {
//         HAL_IncTick();
//     }
//     else if (htim->Instance == TIM2)
//     {
//         timer_driver_on_overflow();
//     }
// }
