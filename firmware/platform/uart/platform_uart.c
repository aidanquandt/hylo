/*---------------------------------------------------------------------------
 * @file    platform_uart.c
 * @brief   UART hardware abstraction layer implementation
 * @details Provides low-level UART hardware access only. No queuing or tasks.
 * 
 * Architecture:
 * - Platform layer: Hardware abstraction (this file - blocking I/O)
 * - Module layer: Task management and queuing (uart_manager module)
 * 
 * Usage:
 *   - Applications use uart_manager_print() or uart_manager_transmit()
 *   - uart_manager task calls platform_uart_transmit_blocking() for HW access
 *   - Direct receive access via platform_uart_receive()
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_uart.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

#define UART_TX_TIMEOUT_MS  100U    // HAL transmit timeout

// UART peripheral handles
extern UART_HandleTypeDef huart4;  // Printing (USB to UART bridge)

#define PLATFORM_UART_PRINT (&huart4)

/*---------------------------------------------------------------------------*/
/* Public Function Implementations                                           */
/*---------------------------------------------------------------------------*/

platform_uart_status_E platform_uart_transmit_blocking(const uint8_t *data, size_t length)
{
    // Validate parameters
    if (data == NULL || length == 0U) {
        return PLATFORM_UART_ERROR;
    }
    
    // Direct HAL call (blocking) - only called by uart_manager task
    HAL_StatusTypeDef status = HAL_UART_Transmit(PLATFORM_UART_PRINT, 
                                                 (uint8_t *)data, 
                                                 length, 
                                                 UART_TX_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_UART_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_UART_TIMEOUT;
    } else {
        return PLATFORM_UART_ERROR;
    }
}

platform_uart_status_E platform_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, size_t length)
{
    // Validate parameters
    if (huart == NULL || data == NULL || length == 0U) {
        return PLATFORM_UART_ERROR;
    }
    
    // Direct HAL call (receive is less commonly used, can enhance later if needed)
    HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, length, UART_TX_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_UART_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_UART_TIMEOUT;
    } else {
        return PLATFORM_UART_ERROR;
    }
}
