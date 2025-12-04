#ifndef PLATFORM_UART_H
#define PLATFORM_UART_H

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/

#include "common.h"
#include "main.h"
#include "usart.h"

/*---------------------------------------------------------------------------*/
/* Typedefs                                                                  */
/*---------------------------------------------------------------------------*/
typedef enum {
    PLATFORM_UART_SUCCESS = 0,
    PLATFORM_UART_ERROR,
    PLATFORM_UART_TIMEOUT
} platform_uart_status_E;

/*---------------------------------------------------------------------------*/
/* Public Function Prototypes (Hardware Abstraction Only)                    */
/*---------------------------------------------------------------------------*/

/**
 * @brief Transmit data over UART (blocking, direct hardware access)
 * @param data pointer to buffer to transmit
 * @param length number of bytes to send
 * @return PLATFORM_UART_SUCCESS if OK, PLATFORM_UART_TIMEOUT or PLATFORM_UART_ERROR on failure
 * @note This function blocks until transmission complete
 * @note Called by uart_manager task - applications should use uart_manager_transmit()
 */
platform_uart_status_E platform_uart_transmit_blocking(const uint8_t *data, size_t length);

/**
 * @brief Receive data over UART (blocking, direct hardware access)
 * @param huart UART handle
 * @param data pointer to buffer
 * @param length number of bytes to read
 * @return PLATFORM_UART_SUCCESS if OK, PLATFORM_UART_TIMEOUT or PLATFORM_UART_ERROR on failure
 * @note This function blocks until reception complete
 */
platform_uart_status_E platform_uart_receive(UART_HandleTypeDef *huart, uint8_t *data, size_t length);

#endif // PLATFORM_UART_H
