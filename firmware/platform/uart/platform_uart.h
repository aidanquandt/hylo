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

/**
 * @brief Start UART RX DMA in circular mode
 * @param buffer Pointer to circular receive buffer
 * @param size Size of buffer in bytes
 * @return PLATFORM_UART_SUCCESS if started, PLATFORM_UART_ERROR on failure
 * @note DMA runs continuously in circular mode until stopped
 * @note Used by uart_manager for command reception
 */
platform_uart_status_E platform_uart_start_rx_dma(uint8_t *buffer, uint16_t size);

/**
 * @brief Get current DMA RX write position
 * @return Current position (0 to buffer_size-1) where DMA will write next byte
 * @note Used for polling DMA buffer in uart_manager
 */
uint16_t platform_uart_get_rx_dma_position(void);

#endif // PLATFORM_UART_H
