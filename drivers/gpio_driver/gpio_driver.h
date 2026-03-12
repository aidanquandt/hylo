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
    GPIO_DRIVER_LOW  = 0,
    GPIO_DRIVER_HIGH = 1
} gpio_driver_state_t;

typedef enum
{
    GPIO_DRIVER_PIN_LED_GREEN
} gpio_driver_pin_t;

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/
void gpio_driver_set_leds(gpio_driver_state_t state);
void gpio_driver_toggle_led_green(void);
gpio_driver_state_t gpio_driver_read_pin(gpio_driver_pin_t pin);
