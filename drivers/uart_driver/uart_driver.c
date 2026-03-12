/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "feature_config.h"
#include "main.h"
#include "stream_buffer.h"
#include "queue.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_TIMEOUT_MS 100U
#define UART_RX_TIMEOUT_MS 100U

#define UART_DRIVER_TX_QUEUE_SIZE 32U
#define UART_DRIVER_TX_MSG_MAX_LENGTH 256U
#define UART_DRIVER_TX_TASK_STACK_SIZE 512U
#define UART_DRIVER_TX_TASK_PRIORITY 3U
#define UART_DRIVER_RX_BUFFER_SIZE 256U
#define UART_DRIVER_LINE_MAX_LENGTH 128U
#define UART_DRIVER_PRINT_BUFFER_SIZE 256U
#define UART_CMD_DELIMITER '\n'

#if (HWREV == 0)
extern UART_HandleTypeDef huart3;
#define UART_DRIVER_PRINT (&huart3)
#define UART_DRIVER_RX (&huart3)
#elif (HWREV == 1)
extern UART_HandleTypeDef huart1;
#define UART_DRIVER_PRINT (&huart1)
#define UART_DRIVER_RX (&huart1)
#endif

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define WIFI_UART (&huart2)
#define WIFI_UART_DMA_RX (&hdma_usart2_rx)

StreamBufferHandle_t rxStream = NULL;
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
uint8_t rx_dma_buf[128] = {0};

/*---------------------------------------------------------------------------
 * Console TX/RX (queue + task + line assembly in driver)
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t data[UART_DRIVER_TX_MSG_MAX_LENGTH];
    uint16_t length;
} uart_driver_tx_message_t;

STATIC QueueHandle_t console_tx_queue           = NULL;
STATIC TaskHandle_t console_tx_task_handle      = NULL;
STATIC volatile uint32_t console_tx_dropped     = 0U;
STATIC volatile uint32_t console_tx_errors     = 0U;
STATIC uint8_t console_tx_dma_buffer[UART_DRIVER_TX_MSG_MAX_LENGTH]
    __attribute__((section(".dma_buffer")));

STATIC uint8_t console_rx_dma_buffer[UART_DRIVER_RX_BUFFER_SIZE]
    __attribute__((section(".dma_buffer")));
STATIC uint8_t console_line_buffer[UART_DRIVER_LINE_MAX_LENGTH];
STATIC uint16_t console_line_length             = 0U;
STATIC uint16_t console_last_checked_pos        = 0U;
STATIC bool console_last_was_cr                = false;
STATIC uint32_t console_rx_overruns             = 0U;
STATIC uart_driver_rx_line_callback_t console_rx_line_callback = NULL;

STATIC void uart_driver_console_tx_task(void* argument);

/*---------------------------------------------------------------------------
 * Private Variables (legacy)
 *---------------------------------------------------------------------------*/
STATIC TaskHandle_t tx_task_to_notify = NULL;

/*---------------------------------------------------------------------------
 * Console TX task and init
 *---------------------------------------------------------------------------*/
STATIC void uart_driver_console_tx_task(void* argument)
{
    (void)argument;
    tx_task_to_notify = xTaskGetCurrentTaskHandle();
    uart_driver_tx_message_t msg;

    for (;;)
    {
        if (xQueueReceive(console_tx_queue, &msg, portMAX_DELAY) != pdPASS)
        {
            continue;
        }
        if (msg.length == 0U || msg.length > UART_DRIVER_TX_MSG_MAX_LENGTH)
        {
            continue;
        }
        memcpy(console_tx_dma_buffer, msg.data, msg.length);
        uart_driver_status_E status =
            uart_driver_transmit_dma(console_tx_dma_buffer, msg.length);
        if (status == UART_DRIVER_SUCCESS)
        {
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0)
            {
                console_tx_errors++;
            }
        }
        else
        {
            console_tx_errors++;
        }
    }
}

void uart_driver_console_init(void)
{
    if (console_tx_queue != NULL)
    {
        return;
    }
    console_tx_queue = xQueueCreate(UART_DRIVER_TX_QUEUE_SIZE, sizeof(uart_driver_tx_message_t));
    (void)xTaskCreate(uart_driver_console_tx_task, "UART_TX", UART_DRIVER_TX_TASK_STACK_SIZE,
                      NULL, UART_DRIVER_TX_TASK_PRIORITY, &console_tx_task_handle);
    console_line_length    = 0U;
    console_last_checked_pos = 0U;
    console_last_was_cr    = false;
    uart_driver_start_rx_dma(console_rx_dma_buffer, UART_DRIVER_RX_BUFFER_SIZE);
}

bool uart_driver_send(const uint8_t* data, size_t length)
{
    if (console_tx_queue == NULL || data == NULL || length == 0U ||
        length > UART_DRIVER_TX_MSG_MAX_LENGTH)
    {
        return false;
    }
    uart_driver_tx_message_t msg;
    msg.length = (uint16_t)length;
    memcpy(msg.data, data, length);
    if (xQueueSend(console_tx_queue, &msg, 0) != pdPASS)
    {
        if (console_tx_dropped < UINT32_MAX)
        {
            console_tx_dropped++;
        }
        return false;
    }
    return true;
}

bool uart_driver_print(const char* format, ...)
{
    if (format == NULL || console_tx_queue == NULL)
    {
        return false;
    }
    char buffer[UART_DRIVER_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (len < 0)
    {
        return false;
    }
    if (len >= (int)sizeof(buffer))
    {
        len = (int)sizeof(buffer) - 1;
    }
    return uart_driver_send((const uint8_t*)buffer, (size_t)len);
}

void uart_driver_register_rx_line_callback(uart_driver_rx_line_callback_t callback)
{
    console_rx_line_callback = callback;
}

void uart_driver_poll_rx(void)
{
    if (console_rx_line_callback == NULL)
    {
        return;
    }
    uint16_t dma_write_pos = uart_driver_get_rx_dma_position();
    while (console_last_checked_pos != dma_write_pos)
    {
        char c = (char)console_rx_dma_buffer[console_last_checked_pos];

        if (c == 0x08 || c == 0x7F)
        {
            if (console_line_length > 0U)
            {
                console_line_length--;
            }
            console_last_was_cr = false;
            console_last_checked_pos = (console_last_checked_pos + 1) % UART_DRIVER_RX_BUFFER_SIZE;
            continue;
        }

        if (c == UART_CMD_DELIMITER || c == '\r')
        {
            if (c == UART_CMD_DELIMITER && console_last_was_cr)
            {
                console_last_was_cr = false;
                console_last_checked_pos =
                    (console_last_checked_pos + 1) % UART_DRIVER_RX_BUFFER_SIZE;
                continue;
            }
            console_last_was_cr = (c == '\r');
            console_line_buffer[console_line_length] = '\0';
            if (console_line_length > 0U)
            {
                console_rx_line_callback((const char*)console_line_buffer, console_line_length);
            }
            console_line_length = 0U;
        }
        else
        {
            console_last_was_cr = false;
            if (console_line_length < UART_DRIVER_LINE_MAX_LENGTH - 1)
            {
                console_line_buffer[console_line_length++] = (uint8_t)c;
            }
            else
            {
                if (console_rx_overruns < UINT32_MAX)
                {
                    console_rx_overruns++;
                }
            }
        }
        console_last_checked_pos =
            (console_last_checked_pos + 1) % UART_DRIVER_RX_BUFFER_SIZE;
    }
}

uint32_t uart_driver_get_tx_queue_count(void)
{
    return (console_tx_queue != NULL) ? (uint32_t)uxQueueMessagesWaiting(console_tx_queue) : 0U;
}

uint32_t uart_driver_get_tx_dropped_count(void)
{
    return console_tx_dropped;
}

uint32_t uart_driver_get_tx_errors(void)
{
    return console_tx_errors;
}

uint32_t uart_driver_get_rx_overruns(void)
{
    return console_rx_overruns;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations (legacy / low-level)
 *---------------------------------------------------------------------------*/

uart_driver_status_E uart_driver_transmit_blocking(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return UART_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(UART_DRIVER_PRINT, (uint8_t*)data, length, UART_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return UART_DRIVER_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return UART_DRIVER_TIMEOUT;
    }
    else
    {
        return UART_DRIVER_ERROR;
    }
}

uart_driver_status_E uart_driver_start_rx_dma(uint8_t* buffer, uint16_t size)
{
    if (buffer == NULL || size == 0U)
    {
        return UART_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(UART_DRIVER_RX, buffer, size);

    if (status == HAL_OK)
    {
        return UART_DRIVER_SUCCESS;
    }
    else
    {
        return UART_DRIVER_ERROR;
    }
}

void uart_driver_register_tx_task(TaskHandle_t task_handle)
{
    tx_task_to_notify = task_handle;
}

uart_driver_status_E uart_driver_transmit_dma(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return UART_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(UART_DRIVER_PRINT, (uint8_t*)data, length);

    if (status == HAL_OK)
    {
        return UART_DRIVER_SUCCESS;
    }
    else if (status == HAL_BUSY)
    {
        return UART_DRIVER_BUSY;
    }
    else
    {
        return UART_DRIVER_ERROR;
    }
}

/*---------------------------------------------------------------------------
 * WIFI
 *---------------------------------------------------------------------------*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == WIFI_UART) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xStreamBufferSendFromISR(rxStream, rx_dma_buf, size, &xHigherPriorityTaskWoken);

        HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
        __HAL_DMA_DISABLE_IT(WIFI_UART_DMA_RX, DMA_IT_HT);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == WIFI_UART) {
        HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
        __HAL_DMA_DISABLE_IT(WIFI_UART_DMA_RX, DMA_IT_HT);
    }
}

void uart_driver_wifi_rx_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(WIFI_UART, rx_dma_buf, sizeof(rx_dma_buf));
    __HAL_DMA_DISABLE_IT(WIFI_UART_DMA_RX, DMA_IT_HT);
}

uart_driver_status_E uart_driver_wifi_transmit_blocking(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U)
    {
        return UART_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(WIFI_UART, (uint8_t*)data, length, UART_TX_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return UART_DRIVER_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return UART_DRIVER_TIMEOUT;
    }
    else
    {
        return UART_DRIVER_ERROR;
    }
}

/*---------------------------------------------------------------------------
 * HAL Callback - Called from DMA TX Complete ISR
 *---------------------------------------------------------------------------*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart == UART_DRIVER_PRINT)
    {
        if (tx_task_to_notify != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(tx_task_to_notify, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

uint16_t uart_driver_get_rx_dma_position(void)
{
    uint16_t dma_counter = (uint16_t)__HAL_DMA_GET_COUNTER(UART_DRIVER_RX->hdmarx);
    uint16_t buffer_size = UART_DRIVER_RX->RxXferSize;
    uint16_t write_pos   = buffer_size - dma_counter;
    return write_pos;
}
