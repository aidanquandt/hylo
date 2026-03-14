#include "uart_driver.h"
#include "FreeRTOS.h"
#include "main.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"
#include <stdbool.h>
#include <string.h>

#define UART_DMA_TIMEOUT_MS 50U

#if (HWREV == 0)
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;
#define CONSOLE_HUART (&huart3)
#define WIFI_HUART    (&huart2)
#elif (HWREV == 1)
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#define CONSOLE_HUART (&huart1)
#define WIFI_HUART    (&huart2)
#endif

typedef struct {
    uint8_t  data[UART_TX_MAX_MSG_LEN];
    uint16_t len;
} uart_msg_t;

typedef struct {
    QueueHandle_t     queue;
    SemaphoreHandle_t complete;
    uint32_t          drop_count;
} uart_tx_ctx_t;

typedef struct {
    StreamBufferHandle_t stream;
    uint8_t             *buf;
    uint16_t             buf_len;
    uint16_t             last_received; /* for circular DMA: only push delta so framing sees one frame per callback */
    volatile uint32_t    drop_count;
} uart_rx_ctx_t;

typedef struct {
    UART_HandleTypeDef *huart;
} uart_port_t;

/* DMA buffers in DMA-accessible RAM (e.g. RAM_D2). DTCM is not reachable by DMA on H7. */
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
static uint8_t uart_rx_dma_buf[UART_COUNT][UART_RX_BUF_LEN];
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
static uint8_t uart_tx_dma_buf[UART_COUNT][UART_TX_MAX_MSG_LEN];

static const uart_port_t uart_ports[UART_COUNT] = {
    [UART_CONSOLE] = { .huart = CONSOLE_HUART },
    [UART_WIFI]    = { .huart = WIFI_HUART },
};

static uart_tx_ctx_t tx_ctx[UART_COUNT];
static uart_rx_ctx_t rx_ctx[UART_COUNT];

static uart_id_t uart_id_from_handle(UART_HandleTypeDef *huart)
{
    for (uart_id_t id = 0; id < UART_COUNT; id++)
        if (uart_ports[id].huart == huart)
            return id;
    return UART_COUNT;
}

static bool uart_hal_transmit_dma(uart_id_t id, const uint8_t *buf, uint16_t len)
{
    if (id >= UART_COUNT || buf == NULL || len == 0)
        return false;
    memcpy(uart_tx_dma_buf[id], buf, len);
    return HAL_UART_Transmit_DMA(uart_ports[id].huart, uart_tx_dma_buf[id], len) == HAL_OK;
}

static bool uart_hal_receive_dma(uart_id_t id)
{
    if (id >= UART_COUNT || rx_ctx[id].buf == NULL || rx_ctx[id].buf_len == 0)
        return false;
    return HAL_UARTEx_ReceiveToIdle_DMA(uart_ports[id].huart,
                                        rx_ctx[id].buf,
                                        rx_ctx[id].buf_len) == HAL_OK;
}

static void uart_driver_tx_cplt_handler(uart_id_t id)
{
    BaseType_t woken = pdFALSE;
    if (tx_ctx[id].complete != NULL)
        xSemaphoreGiveFromISR(tx_ctx[id].complete, &woken);
    portYIELD_FROM_ISR(woken);
}

static void uart_rx_push(uart_rx_ctx_t *rx, const uint8_t *p, uint16_t len, BaseType_t *woken)
{
    if (len == 0U)
        return;
    size_t w = xStreamBufferSendFromISR(rx->stream, p, len, woken);
    if (w < (size_t)len)
        rx->drop_count += (uint32_t)(len - (uint16_t)w);
}

static void uart_driver_rx_cplt_handler(uart_id_t id, uint16_t received)
{
    BaseType_t woken = pdFALSE;
    uart_rx_ctx_t *rx = &rx_ctx[id];
    uint16_t last = rx->last_received;
    uint16_t buf_len = rx->buf_len;

    if (received > last)
        uart_rx_push(rx, rx->buf + last, received - last, &woken);
    else if (received < last) {
        uart_rx_push(rx, rx->buf + last, buf_len - last, &woken);
        uart_rx_push(rx, rx->buf, received, &woken);
    }

    rx->last_received = received;
    if (uart_hal_receive_dma(id))
        rx->last_received = 0;
    portYIELD_FROM_ISR(woken);
}

static void uart_driver_drain_task(void *arg)
{
    uart_id_t id = (uart_id_t)(uintptr_t)arg;
    uart_msg_t msg;

    for (;;)
    {
        xQueueReceive(tx_ctx[id].queue, &msg, portMAX_DELAY);
        if (uart_hal_transmit_dma(id, msg.data, msg.len))
        {
            if (!xSemaphoreTake(tx_ctx[id].complete, pdMS_TO_TICKS(UART_DMA_TIMEOUT_MS)))
                tx_ctx[id].drop_count++;
        }
        else
            tx_ctx[id].drop_count++;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_id_t id = uart_id_from_handle(huart);
    if (id >= UART_COUNT)
        return;
    uart_driver_tx_cplt_handler(id);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uart_id_t id = uart_id_from_handle(huart);
    if (id >= UART_COUNT)
        return;
    uart_driver_rx_cplt_handler(id, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_id_t id = uart_id_from_handle(huart);
    if (id >= UART_COUNT)
        return;
    (void)uart_hal_receive_dma(id);
}

void uart_driver_init(void)
{
    for (uart_id_t id = 0; id < UART_COUNT; id++)
    {
        tx_ctx[id].complete = xSemaphoreCreateBinary();
        tx_ctx[id].queue    = xQueueCreate(UART_TX_QUEUE_DEPTH, sizeof(uart_msg_t));
        tx_ctx[id].drop_count = 0;
        configASSERT(tx_ctx[id].complete);
        configASSERT(tx_ctx[id].queue);

        rx_ctx[id].stream        = xStreamBufferCreate(UART_RX_STREAM_SIZE, 1);
        rx_ctx[id].buf           = uart_rx_dma_buf[id];
        rx_ctx[id].buf_len       = UART_RX_BUF_LEN;
        rx_ctx[id].last_received = 0;
        rx_ctx[id].drop_count    = 0;
        configASSERT(rx_ctx[id].stream);

        configASSERT(uart_hal_receive_dma(id));

        xTaskCreate(uart_driver_drain_task, id == UART_CONSOLE ? "UART_tx_console" : "UART_tx_wifi",
                    256, (void *)(uintptr_t)id, tskIDLE_PRIORITY + 2, NULL);
    }
}

void uart_driver_rx_start(uart_id_t id)
{
    if (id >= UART_COUNT)
        return;
    (void)uart_hal_receive_dma(id);
}

StreamBufferHandle_t uart_driver_get_rx_stream(uart_id_t id)
{
    if (id >= UART_COUNT)
        return NULL;
    return rx_ctx[id].stream;
}

uint32_t uart_driver_get_drop_count(uart_id_t id)
{
    if (id >= UART_COUNT)
        return 0U;
    uint32_t n_rx, n_tx;
    taskENTER_CRITICAL();
    n_rx = rx_ctx[id].drop_count;
    rx_ctx[id].drop_count = 0;
    n_tx = tx_ctx[id].drop_count;
    tx_ctx[id].drop_count = 0;
    taskEXIT_CRITICAL();
    return n_rx + n_tx;
}

void uart_driver_transmit(uart_id_t id, const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0 || id >= UART_COUNT)
        return;

    if (len > UART_TX_MAX_MSG_LEN)
    {
        taskENTER_CRITICAL();
        tx_ctx[id].drop_count++;
        taskEXIT_CRITICAL();
        return;
    }

    uart_msg_t msg;
    msg.len = (uint16_t)len;
    memcpy(msg.data, buf, msg.len);
    xQueueSend(tx_ctx[id].queue, &msg, portMAX_DELAY);
}
