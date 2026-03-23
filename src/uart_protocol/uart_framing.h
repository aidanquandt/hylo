#ifndef UART_FRAMING_H
#define UART_FRAMING_H

#include <stdint.h>
#include <stddef.h>

typedef enum {
    PROTOCOL_UART_DEST_CONSOLE,
    PROTOCOL_UART_DEST_WIFI
} protocol_uart_destination_e;

/* Context structure to allow multiple independent decoders */
typedef struct {
    uint8_t cobs_buf[128]; 
    size_t cobs_len;
    protocol_uart_destination_e source;  /* Track which source this decoder is for */
} uart_decoder_t;

void protocol_set_uart_destination(protocol_uart_destination_e destination);
protocol_uart_destination_e protocol_get_uart_destination(void);
protocol_uart_destination_e protocol_get_decoder_source(void);
void protocol_send_frame(uint16_t msg_id, const uint8_t *payload, size_t len);

/**
 * @brief Feed bytes into a specific decoder context.
 * @param decoder Pointer to the context (console or wifi)
 * @param buf Incoming data bytes
 * @param len Number of bytes
 */
void uart_framing_feed(uart_decoder_t *decoder, const uint8_t *buf, size_t len);

#endif