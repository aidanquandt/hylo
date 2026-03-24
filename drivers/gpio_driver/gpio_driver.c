/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "gpio_driver.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#if (HWREV == 0)
#include "stm32h7xx_nucleo.h"
#endif
#if (HWREV == 1)
#include "gpio.h"
#endif

/*---------------------------------------------------------------------------
 * External weak callback - can be overridden by higher layers
 *---------------------------------------------------------------------------*/
__attribute__((weak)) void gpio_driver_uwb_irq_callback(void)
{
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void gpio_driver_set_leds(gpio_driver_state_t state)
{
    if (state == GPIO_DRIVER_HIGH)
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

void gpio_driver_toggle_led_green(void)
{
#if (HWREV == 0)
    BSP_LED_Toggle(LED_GREEN);
#endif
}

void gpio_driver_esp_set(void)
{
    #if (HWREV == 1)
    HAL_GPIO_WritePin(ESP_nRST_GPIO_Port, ESP_nRST_Pin, GPIO_PIN_SET);
    #endif
}

void gpio_driver_esp_reset(void)
{   
    #if (HWREV == 1)
    HAL_GPIO_WritePin(ESP_nRST_GPIO_Port, ESP_nRST_Pin, GPIO_PIN_RESET);
    #endif
}

void gpio_driver_led_1_on(void)
{   
    #if (HWREV == 1)
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    #endif
}

void gpio_driver_led_1_off(void)
{   
    #if (HWREV == 1)
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    #endif
}

gpio_driver_state_t gpio_driver_read_pin(gpio_driver_pin_t pin)
{
#if (HWREV == 0)
    GPIO_TypeDef* port;
    uint16_t pin_num;

    switch (pin)
    {
        case GPIO_DRIVER_PIN_LED_GREEN:
            port    = LED1_GPIO_PORT;
            pin_num = LED1_PIN;
            break;
        default:
            port    = NULL;
            pin_num = 0;
            return GPIO_DRIVER_LOW;
    }

    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin_num);
    return (state == GPIO_PIN_SET) ? GPIO_DRIVER_HIGH : GPIO_DRIVER_LOW;
#else
    return GPIO_DRIVER_LOW;
#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if (HWREV == 0 || HWREV == 1)
    if (GPIO_Pin == UWB_IRQ_Pin)
    {
        gpio_driver_uwb_irq_callback();
    }
#endif
}
