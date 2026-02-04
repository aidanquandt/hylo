/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_os.h"
#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define PLATFORM_OS_MAX_US_DELAY 10000U // 10ms max for scheduler suspension

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void platform_os_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void platform_os_delay_us_blocking(uint32_t delay_us)
{
    if (delay_us == 0)
    {
        return;
    }

    // Check if we're in an ISR context
    bool from_isr = (xPortIsInsideInterrupt() == pdTRUE);

    // If in ISR or delay is very short, just busy-wait without scheduler suspension
    if (from_isr || delay_us <= 100)
    {
        uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
        uint32_t start  = DWT->CYCCNT;

        while ((DWT->CYCCNT - start) < cycles)
        {
        }
        return;
    }

    // For longer delays in task context, suspend scheduler to prevent context switches
    if (delay_us > PLATFORM_OS_MAX_US_DELAY)
    {
        vTaskDelay(pdMS_TO_TICKS((delay_us + 999) / 1000));
        return;
    }

    vTaskSuspendAll();

    uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
    uint32_t start  = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }

    xTaskResumeAll();
}

platform_os_critical_state_t platform_os_critical_enter(void)
{
    platform_os_critical_state_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void platform_os_critical_exit(platform_os_critical_state_t state)
{
    __set_PRIMASK(state);
}
