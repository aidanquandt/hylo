/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_os.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Private Definitions
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

uint32_t platform_os_gettick(void)
{
    return HAL_GetTick();
}

void platform_os_delay_us_blocking(uint32_t delay_us)
{
    if (delay_us == 0)
    {
        return;
    }

    // Check if we're in an ISR context
    bool from_isr = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U;

    // If in ISR or delay is very short, just busy-wait without scheduler suspension
    if (from_isr || delay_us <= 100)
    {
        uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
        uint32_t start = DWT->CYCCNT;

        while ((DWT->CYCCNT - start) < cycles)
        {
        }
        return;
    }

    // For longer delays in task context, suspend scheduler to prevent context switches
    if (delay_us > PLATFORM_OS_MAX_US_DELAY)
    {
        platform_os_delay_ms((delay_us + 999) / 1000);
        return;
    }

    vTaskSuspendAll();

    uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;
    uint32_t start = DWT->CYCCNT;

    while ((DWT->CYCCNT - start) < cycles)
    {
    }

    xTaskResumeAll();
}

void platform_os_delay_ms(uint32_t delay_ms)
{
    osDelay(delay_ms);
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

platform_os_mutex_t platform_os_mutex_create(void)
{
    return (platform_os_mutex_t)xSemaphoreCreateMutex();
}

bool platform_os_mutex_take(platform_os_mutex_t mutex, uint32_t timeout_ms)
{
    if (mutex == NULL)
    {
        return false;
    }

    TickType_t ticks = (timeout_ms == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake((SemaphoreHandle_t)mutex, ticks) == pdTRUE;
}

void platform_os_mutex_give(platform_os_mutex_t mutex)
{
    if (mutex != NULL)
    {
        xSemaphoreGive((SemaphoreHandle_t)mutex);
    }
}
