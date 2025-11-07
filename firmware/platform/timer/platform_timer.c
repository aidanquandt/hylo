/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_timer.h"
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
// Hardware timer configuration: TIM5 with prescaler 2399 at 240MHz
// Results in: 240MHz / 2400 = 100kHz timer clock = 10µs per tick
#define PLATFORM_TIMER_US_PER_TICK  (10U)

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

uint32_t platform_get_timestamp(void)
{
    return TIM5->CNT;
}

uint32_t platform_get_elapsed_us(uint32_t start_timestamp, uint32_t end_timestamp)
{
    // Handle 32-bit counter wraparound with unsigned arithmetic
    uint32_t elapsed_ticks = end_timestamp - start_timestamp;
    return elapsed_ticks * PLATFORM_TIMER_US_PER_TICK;
}