#include "uart_framing.h"
#include "protocol_dispatch.h"
#include "uart_driver.h"
#include <stdbool.h>
#include <string.h>

#define PROTOCOL_VERSION    1U
#define PROTOCOL_HEADER_LEN 7U
#define PROTOCOL_MAX_PAYLOAD 110U
#define UART_TX_MAX_LEN     128U

static volatile protocol_uart_destination_e s_protocol_uart_destination = PROTOCOL_UART_DEST_CONSOLE;

/* Track which decoder context is currently processing (for bidirectional routing) */
static protocol_uart_destination_e s_current_decoder_source = PROTOCOL_UART_DEST_CONSOLE;

static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFFU;
    while (len--) {
        crc ^= (uint16_t)*data++ << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000U) crc = (crc << 1) ^ 0x1021U;
            else crc = crc << 1;
        }
    }
    return crc;
}

static size_t cobs_encode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_cap) {
    if (dst_cap < 2) return 0;
    size_t out = 0, run_start = 0;
    while (run_start <= src_len) {
        if (run_start == src_len) {
            if (out + 1 > dst_cap) return 0;
            dst[out++] = 0x00;
            return out;
        }
        size_t block_len = 0;
        while (run_start + block_len < src_len && src[run_start + block_len] != 0 && block_len < 254U)
            block_len++;
        if (out + 1 + block_len > dst_cap) return 0;
        dst[out++] = (uint8_t)(block_len + 1);
        for (size_t k = 0; k < block_len; k++) dst[out++] = src[run_start + k];
        run_start += block_len;
        if (run_start < src_len && src[run_start] == 0) run_start++;
    }
    return out;
}

static size_t cobs_decode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_cap) {
    size_t out = 0;
    for (size_t i = 0; i < src_len; ) {
        uint8_t code = src[i++];
        if (code == 0x00) return out;
        if (code > 1) {
            size_t copy = (size_t)(code - 1);
            if (i + copy > src_len || out + copy > dst_cap) return 0;
            for (size_t k = 0; k < copy; k++) dst[out++] = src[i++];
        }
        if (code != 255 && out < dst_cap) {
            if (code == 1) dst[out++] = 0;
            else if (i < src_len && src[i] != 0x00) dst[out++] = 0;
        }
    }
    return out;
}

static void try_dispatch_cobs_block(const uart_decoder_t *decoder, const uint8_t *block, size_t block_len) {
    uint8_t raw_buf[PROTOCOL_HEADER_LEN + PROTOCOL_MAX_PAYLOAD];
    size_t raw_len = cobs_decode(block, block_len, raw_buf, sizeof(raw_buf));
    
    if (raw_len < PROTOCOL_HEADER_LEN || raw_buf[0] != PROTOCOL_VERSION) return;

    uint16_t payload_len = (uint16_t)raw_buf[3] | ((uint16_t)raw_buf[4] << 8);
    if (raw_len != (size_t)(7U + payload_len)) return;

    uint16_t crc_rx = (uint16_t)raw_buf[5 + payload_len] | ((uint16_t)raw_buf[6 + payload_len] << 8);
    if (crc_rx != crc16_ccitt(raw_buf, 5 + payload_len)) return;

    /* Set the current decoder source for bidirectional routing */
    s_current_decoder_source = decoder->source;
    
    uint16_t msg_id = (uint16_t)raw_buf[1] | ((uint16_t)raw_buf[2] << 8);
    protocol_dispatch(msg_id, raw_buf + 5, payload_len);
}

void protocol_set_uart_destination(protocol_uart_destination_e destination) {
    s_protocol_uart_destination = destination;
}

protocol_uart_destination_e protocol_get_uart_destination(void) {
    return s_protocol_uart_destination;
}

protocol_uart_destination_e protocol_get_decoder_source(void) {
    return s_current_decoder_source;
}

void protocol_send_frame(uint16_t msg_id, const uint8_t *payload, size_t len) {
    if (payload == NULL || len > PROTOCOL_MAX_PAYLOAD) return;
    uint8_t raw[PROTOCOL_HEADER_LEN + PROTOCOL_MAX_PAYLOAD];
    raw[0] = (uint8_t)PROTOCOL_VERSION;
    raw[1] = (uint8_t)(msg_id & 0xFF);
    raw[2] = (uint8_t)(msg_id >> 8);
    raw[3] = (uint8_t)(len & 0xFF);
    raw[4] = (uint8_t)(len >> 8);
    memcpy(raw + 5, payload, len);
    uint16_t crc = crc16_ccitt(raw, 5 + len);
    raw[5 + len] = (uint8_t)(crc & 0xFF);
    raw[6 + len] = (uint8_t)(crc >> 8);

    uint8_t cobs_buf[UART_TX_MAX_LEN];
    size_t cobs_len = cobs_encode(raw, 7 + len, cobs_buf, sizeof(cobs_buf));
    if (cobs_len == 0) return;

    uart_id_t ch = (s_protocol_uart_destination == PROTOCOL_UART_DEST_WIFI) ? UART_WIFI : UART_CONSOLE;
    uart_driver_transmit(ch, cobs_buf, cobs_len);
}

void uart_framing_feed(uart_decoder_t *decoder, const uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (buf[i] == 0x00) {
            if (decoder->cobs_len > 0) {
                try_dispatch_cobs_block(decoder, decoder->cobs_buf, decoder->cobs_len);
                decoder->cobs_len = 0;
            }
        } else {
            if (decoder->cobs_len < sizeof(decoder->cobs_buf)) {
                decoder->cobs_buf[decoder->cobs_len++] = buf[i];
            } else {
                decoder->cobs_len = 0;
            }
        }
    }
}