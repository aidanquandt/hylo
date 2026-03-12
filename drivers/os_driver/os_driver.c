/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "os_driver.h"
#include "FreeRTOS.h"
#include "stm32h7xx_hal.h"
#include "task.h"

#define OS_DRIVER_MAX_US_DELAY 10000U

void os_driver_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void os_driver_delay_us_blocking(uint32_t delay_us)
{
    if (delay_us == 0)
        return;

    bool from_isr = (xPortIsInsideInterrupt() == pdTRUE);

    if (from_isr || delay_us <= 100)
    {
        uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
        uint32_t start  = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles)
            ;
        return;
    }

    if (delay_us > OS_DRIVER_MAX_US_DELAY)
    {
        vTaskDelay(pdMS_TO_TICKS((delay_us + 999) / 1000));
        return;
    }

    vTaskSuspendAll();
    uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
    uint32_t start  = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles)
        ;
    xTaskResumeAll();
}

os_driver_critical_state_t os_driver_critical_enter(void)
{
    os_driver_critical_state_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void os_driver_critical_exit(os_driver_critical_state_t state)
{
    __set_PRIMASK(state);
}
