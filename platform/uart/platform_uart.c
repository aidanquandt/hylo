/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_uart.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "task.h"
#include "stream_buffer.h"
#include "queue.h"
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_TIMEOUT_MS 100U
#define UART_RX_TIMEOUT_MS 100U

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#if (HWREV == 0)
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;
#else
extern UART_HandleTypeDef huart1;
#endif

#if (HWREV == 0)
    #if FEATURE_USE_USART3
    #define PLATFORM_UART_PRINT (&huart3)
    #define PLATFORM_UART_RX (&huart3)
    #else
    #define PLATFORM_UART_PRINT (&huart4)
    #define PLATFORM_UART_RX (&huart4)
    #endif
#else
    #define PLATFORM_UART_PRINT (&huart1)
    #define PLATFORM_UART_RX (&huart1)
#endif

StreamBufferHandle_t rxStream = NULL;
// DMA buffer in D2 SRAM (non-cacheable region on STM32H7)
// Use section attribute to place in SRAM2/D2 domain
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
uint8_t rx_dma_buf[128] = {0};

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
 * WIFI
 *---------------------------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Push bytes into stream buffer (DMA → Task)
        xStreamBufferSendFromISR(rxStream, rx_dma_buf, size, &xHigherPriorityTaskWoken);

        // Re-arm DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, sizeof(rx_dma_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, sizeof(rx_dma_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
}

void wifi_rx_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, sizeof(rx_dma_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

platform_uart_status_E wifi_uart_transmit_blocking(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return PLATFORM_UART_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(&huart2, (uint8_t*)data, length, UART_TX_TIMEOUT_MS);

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
