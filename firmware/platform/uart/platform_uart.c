/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_uart.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_TIMEOUT_MS 100U
#define UART_RX_TIMEOUT_MS 100U

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

#if FEATURE_USE_USART3
#define PLATFORM_UART_PRINT (&huart3)
#define PLATFORM_UART_RX (&huart3)
#else
#define PLATFORM_UART_PRINT (&huart4)
#define PLATFORM_UART_RX (&huart4)
#endif

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC TaskHandle_t tx_task_to_notify = NULL;

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

void platform_uart_register_tx_task(TaskHandle_t task_handle)
{
    // Validate task handle (can be NULL to unregister)
    tx_task_to_notify = task_handle;
}

platform_uart_status_E platform_uart_transmit_dma(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return PLATFORM_UART_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(PLATFORM_UART_PRINT, (uint8_t*)data, length);

    if (status == HAL_OK)
    {
        return PLATFORM_UART_SUCCESS;
    }
    else if (status == HAL_BUSY)
    {
        return PLATFORM_UART_BUSY;
    }
    else
    {
        return PLATFORM_UART_ERROR;
    }
}

/*---------------------------------------------------------------------------
 * HAL Callback - Called from DMA TX Complete ISR
 *---------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    // Only handle our UART
    if (huart == PLATFORM_UART_PRINT)
    {
        // Notify the TX task that transmission is complete
        if (tx_task_to_notify != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(tx_task_to_notify, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
uint16_t platform_uart_get_rx_dma_position(void)
{
    uint16_t dma_counter = (uint16_t)__HAL_DMA_GET_COUNTER(PLATFORM_UART_RX->hdmarx);
    uint16_t buffer_size = PLATFORM_UART_RX->RxXferSize;
    uint16_t write_pos   = buffer_size - dma_counter;
    return write_pos;
}
