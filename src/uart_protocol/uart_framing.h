/* UART protocol framing: COBS + CRC16. */
#pragma once

#include <stddef.h>
#include <stdint.h>

void protocol_set_endpoint(uint8_t port);
uint8_t protocol_get_endpoint(void);

/* Send a raw frame (msg_id + payload). Framing layer builds header, CRC, COBS, and calls uart_driver_transmit. */
void protocol_send_frame(uint16_t msg_id, const uint8_t *payload, size_t len);

/* Feed received bytes from the UART RX stream. On complete COBS frame: verify CRC, then call protocol_dispatch. */
void uart_framing_feed(uint8_t port, const uint8_t *buf, size_t len);
