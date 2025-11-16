/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_timer.h"
#include "main.h"
#include "cmsis_os.h"

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

// nonblocking for rtos tasks? aidan confirm this
void platform_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

// todo: dedicate timer to this, dont use the dwt?
void platform_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    
    while ((DWT->CYCCNT - start) < cycles) {
        // Busy-wait for precise microsecond timing
    }
}