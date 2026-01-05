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
// Hardware timer selection: TIM5 chosen for 32-bit counter and availability
// Configuration: prescaler 2749 at 275MHz APB1 timer clock
// Results in: 275MHz / 2750 = 100kHz timer clock
// Each timer tick = 10µs
#define PLATFORM_TIMER_US_PER_TICK (10U) // Microseconds per timer tick
#define PLATFORM_TIMESTAMP_TIMER htim5   // Hardware timer used for timestamps

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
    // Handle 32-bit counter wraparound with unsigned arithmetic
    uint32_t elapsed_ticks = end_timestamp - start_timestamp;
    return elapsed_ticks * PLATFORM_TIMER_US_PER_TICK;
}