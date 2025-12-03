/*---------------------------------------------------------------------------
 * @file    platform_uart.c
 * @brief   Generic UART hardware abstraction layer implementation
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_uart.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

// UART peripheral handles
extern UART_HandleTypeDef huart4;  // Printing (USB to UART bridge)

#define PLATFORM_UART_PRINT (&huart4)

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC volatile HAL_StatusTypeDef last_hal_status = HAL_OK;  // Debug: store last HAL status

/*---------------------------------------------------------------------------*/
/* Public Function Implementations                                           */
/*---------------------------------------------------------------------------*/

platform_uart_status_E platform_uart_transmit(UART_HandleTypeDef *huart, const uint8_t *data, size_t length)
{

    if (huart == NULL || data == NULL || length == 0)
        return PLATFORM_UART_ERROR;

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t *)data, length, HAL_MAX_DELAY);
    last_hal_status = status;

    if (status == HAL_OK)
        return PLATFORM_UART_SUCCESS;
    else if (status == HAL_TIMEOUT)
        return PLATFORM_UART_TIMEOUT;
    else
        return PLATFORM_UART_ERROR;
}

platform_uart_status_E platform_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, size_t length)
{

    if (huart == NULL || data == NULL || length == 0)
        return PLATFORM_UART_ERROR;

    HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, length, HAL_MAX_DELAY);
    last_hal_status = status;

    if (status == HAL_OK)
        return PLATFORM_UART_SUCCESS;
    else if (status == HAL_TIMEOUT)
        return PLATFORM_UART_TIMEOUT;
    else
        return PLATFORM_UART_ERROR;
}