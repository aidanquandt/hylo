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
// Configuration: prescaler 27 at 275MHz APB1 timer clock
// Results in: 275MHz / 28 = 9.821MHz timer clock
// Each timer tick = 101.818 ns (~0.102 µs)
// Max measurement time before wraparound: 2^32 ticks = 437.89 seconds (~7.3 minutes)
#define PLATFORM_TIMER_NS_PER_TICK (101.81818182f) // Nanoseconds per timer tick
#define PLATFORM_TIMESTAMP_TIMER htim5             // Hardware timer used for timestamps

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
    // Convert from ticks to microseconds: ticks × 101.818ns = ticks × 0.101818µs
    return (uint32_t)((float)elapsed_ticks * (PLATFORM_TIMER_NS_PER_TICK / 1000.0f));
}