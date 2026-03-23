#include "uart_framing.h"
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

#define RX_CHUNK_SIZE  64
#define TASK_STACK     384
#define TASK_PRIORITY  (tskIDLE_PRIORITY + 1)

/* Independent decoder for the Console UART */
static uart_decoder_t console_decoder = {0};

static void uart_protocol_task_fn(void *arg) {
    uint8_t buf[RX_CHUNK_SIZE];
    StreamBufferHandle_t stream = uart_driver_get_rx_stream(UART_CONSOLE);
    
    for (;;) {
        size_t received = xStreamBufferReceive(stream, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (received > 0) {
            /* Feed the console's private decoder context */
            uart_framing_feed(&console_decoder, buf, received);
        }
    }
}

void uart_protocol_task_start(void) {
    /* Initialize console decoder source */
    console_decoder.source = PROTOCOL_UART_DEST_CONSOLE;
    xTaskCreate(uart_protocol_task_fn, "uart_proto", TASK_STACK, NULL, TASK_PRIORITY, NULL);
}