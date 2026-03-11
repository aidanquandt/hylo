/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_gpio.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#if (HWREV == 0)
#include "stm32h7xx_nucleo.h"
#endif

/*---------------------------------------------------------------------------
 * External weak callback - can be overridden by higher layers
 *---------------------------------------------------------------------------*/
__attribute__((weak)) void platform_gpio_uwb_irq_callback(void)
{
    // Default empty implementation - override in uwb_port or app layer
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void platform_gpio_set_leds(platform_gpio_state_t state)
{
    if (state == PLATFORM_GPIO_HIGH)
    {
#if (HWREV == 0)
        BSP_LED_On(LED_GREEN);
        BSP_LED_On(LED_RED);
        BSP_LED_On(LED_YELLOW);
#endif
    }
    else
    {
#if (HWREV == 0)
        BSP_LED_Off(LED_GREEN);
        BSP_LED_Off(LED_RED);
        BSP_LED_Off(LED_YELLOW);
#endif
    }
}

void platform_gpio_toggle_led_green(void)
{
#if (HWREV == 0)
    BSP_LED_Toggle(LED_GREEN);
#endif
}

platform_gpio_state_t platform_gpio_read_pin(platform_gpio_pin_t pin)
{   
    #if (HWREV == 0)
    GPIO_TypeDef* port;
    uint16_t pin_num;

    switch (pin)
    {
        case PLATFORM_GPIO_PIN_LED_GREEN:
            port    = LED1_GPIO_PORT;
            pin_num = LED1_PIN;
            break;
        default:
            port    = NULL;
            pin_num = 0;
            return PLATFORM_GPIO_LOW;
    }

    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin_num);

    return (state == GPIO_PIN_SET) ? PLATFORM_GPIO_HIGH : PLATFORM_GPIO_LOW;
#else
    return PLATFORM_GPIO_LOW; // Default to low if HWREV is not 0
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if (HWREV == 0 || HWREV == 1)
    if (GPIO_Pin == UWB_IRQ_Pin)
    {
        platform_gpio_uwb_irq_callback();
    }
#endif
}