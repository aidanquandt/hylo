/*
 * UART protocol RX task: drain uart_driver RX stream, feed framing decoder.
 * On complete frame the framing layer calls protocol_dispatch.
 */
#include "uart_framing.h"
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include <stddef.h>
#include <string.h>

#define RX_CHUNK_SIZE  64
#define TASK_STACK     1024 /* 4KB: deep tx chain (protocol_send_frame=245B + uart_driver_transmit=130B + protocol_dispatch 170+ cases) */
#define TASK_PRIORITY  (tskIDLE_PRIORITY + 1)

static void uart_protocol_task_fn(void *arg)
{
    (void)arg;
    uint8_t buf[RX_CHUNK_SIZE];
    StreamBufferHandle_t stream = uart_driver_get_rx_stream(UART_CONSOLE);
    if (stream == NULL)
        return;
    for (;;) {
        size_t received = xStreamBufferReceive(stream, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (received > 0)
            uart_framing_feed(UART_CONSOLE, buf, received);
    }
}

void uart_protocol_task_start(void)
{
    StreamBufferHandle_t stream = uart_driver_get_rx_stream(UART_CONSOLE);
    if (stream != NULL)
        xTaskCreate(uart_protocol_task_fn, "uart_proto", TASK_STACK, NULL, TASK_PRIORITY, NULL);
}
