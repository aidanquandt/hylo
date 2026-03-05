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
#define PLATFORM_TIMESTAMP_TIMER   htim5

extern TIM_HandleTypeDef PLATFORM_TIMESTAMP_TIMER;
extern TIM_HandleTypeDef htim2;

/* High 32 bits of 64-bit time (TIM2 overflow count). Incremented in ISR. */
static volatile uint32_t s_overflow_count;

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

void platform_timer_on_overflow(void)
{
    s_overflow_count++;
}

uint64_t platform_get_time_us(void)
{
    uint32_t o1 = s_overflow_count;
    uint32_t c  = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t o2 = s_overflow_count;
    if (o1 != o2)
    {
        c = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    }
    return ((uint64_t)o2 << 32) | c;
}

uint32_t platform_get_time_ms(void)
{
    return (uint32_t)(platform_get_time_us() / 1000ULL);
}