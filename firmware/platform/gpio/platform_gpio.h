#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/
typedef enum
{
    PLATFORM_GPIO_LOW  = 0,
    PLATFORM_GPIO_HIGH = 1
} platform_gpio_state_t;

typedef enum
{
    PLATFORM_GPIO_PIN_LED_GREEN
} platform_gpio_pin_t;

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/
void platform_gpio_toggle_led_green(void);

platform_gpio_state_t platform_gpio_read_pin(platform_gpio_pin_t pin);