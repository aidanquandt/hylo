/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_uart.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_TIMEOUT_MS 100U
#define UART_RX_TIMEOUT_MS 100U

extern UART_HandleTypeDef huart4;

#define PLATFORM_UART_PRINT (&huart4)
#define PLATFORM_UART_RX (&huart4)

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

platform_uart_status_E platform_uart_transmit_blocking(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return PLATFORM_UART_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(PLATFORM_UART_PRINT, (uint8_t*)data, length, UART_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return PLATFORM_UART_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return PLATFORM_UART_TIMEOUT;
    }
    else
    {
        return PLATFORM_UART_ERROR;
    }
}

platform_uart_status_E platform_uart_receive(UART_HandleTypeDef* huart, uint8_t* data,
                                             size_t length)
{
    if (huart == NULL || data == NULL || length == 0U)
    {
        return PLATFORM_UART_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, length, UART_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return PLATFORM_UART_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return PLATFORM_UART_TIMEOUT;
    }
    else
    {
        return PLATFORM_UART_ERROR;
    }
}

platform_uart_status_E platform_uart_start_rx_dma(uint8_t* buffer, uint16_t size)
{
    if (buffer == NULL || size == 0U)
    {
        return PLATFORM_UART_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(PLATFORM_UART_RX, buffer, size);

    if (status == HAL_OK)
    {
        return PLATFORM_UART_SUCCESS;
    }
    else
    {
        return PLATFORM_UART_ERROR;
    }
}

uint16_t platform_uart_get_rx_dma_position(void)
{
    uint16_t dma_counter = (uint16_t)__HAL_DMA_GET_COUNTER(PLATFORM_UART_RX->hdmarx);
    uint16_t buffer_size = PLATFORM_UART_RX->RxXferSize;
    uint16_t write_pos   = buffer_size - dma_counter;
    return write_pos;
}
