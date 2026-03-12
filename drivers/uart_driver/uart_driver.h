#pragma once
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "stream_buffer.h"

#define UART_TX_MAX_MSG_LEN 128U
#define UART_TX_QUEUE_DEPTH 8U
#define UART_RX_BUF_LEN     128U
#define UART_RX_STREAM_SIZE 512U

typedef enum {
    UART_CONSOLE = 0,
    UART_WIFI    = 1,
    UART_COUNT
} uart_id_t;

void uart_driver_init(void);
void uart_driver_transmit(uart_id_t id, const uint8_t *buf, size_t len);
uint32_t uart_driver_get_drop_count(uart_id_t id);
StreamBufferHandle_t uart_driver_get_rx_stream(uart_id_t id);
void uart_driver_rx_start(uart_id_t id);
