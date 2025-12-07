/*---------------------------------------------------------------------------
 * @file    platform_os.c
 * @brief   Operating system abstraction implementation
 *---------------------------------------------------------------------------*/

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
    // Enable DWT cycle counter for precise microsecond delays
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable trace
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
    DWT->CYCCNT = 0;                                // Reset counter
}

uint32_t platform_os_gettick(void)
{
    return HAL_GetTick();
}

void platform_os_delay_us_blocking(uint32_t delay_us)
{
    // Bounds check: prevent excessively long scheduler suspension
    if (delay_us == 0)
    {
        return;
    }

    if (delay_us > PLATFORM_OS_MAX_US_DELAY)
    {
        // For long delays, fall back to millisecond delay with yielding
        platform_os_delay_ms((delay_us + 999) / 1000); // Round up to ms
        return;
    }

    vTaskSuspendAll(); // Suspend scheduler to prevent task switches

    // Calculate number of CPU cycles for the delay
    // SystemCoreClock is CPU frequency in Hz
    uint32_t cycles = (SystemCoreClock / 1000000UL) * delay_us;

    // Suspend scheduler to prevent task preemption during delay
    // Interrupts remain enabled so SysTick and peripherals continue working

    uint32_t start = DWT->CYCCNT;

    // Busy-wait until cycles elapsed
    // Subtraction handles 32-bit counter wraparound correctly
    while ((DWT->CYCCNT - start) < cycles)
    {
        // Spin
    }

    xTaskResumeAll();
}

void platform_os_delay_ms(uint32_t delay_ms)
{
    // Yield to RTOS scheduler
    osDelay(delay_ms); // RTOS-aware delay that yields to other tasks
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
