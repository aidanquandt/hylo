/*
 * UART protocol framing: COBS + CRC16-CCITT.
 * protocol_send_frame() builds header+payload+CRC, COBS-encodes, calls uart_driver_transmit.
 * uart_framing_feed() accepts streamed bytes, decodes COBS, validates CRC, calls protocol_dispatch.
 */
#include "uart_framing.h"
#include "protocol_dispatch.h"
#include "protocol_ids.h"
#include "uart_driver.h"
#include "semphr.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define PROTOCOL_VERSION  1U
#define PROTOCOL_HEADER_LEN 7U  /* version(1) + msg_id(2) + payload_len(2) + crc16(2) */
#define PROTOCOL_MAX_PAYLOAD 110U
#define UART_TX_MAX_LEN 128U

/* Static variable to track which port is currently "Active" for telemetry */
static uart_id_t s_active_proto_port = UART_CONSOLE;
static volatile bool s_dispatch_route_active;
static volatile uart_id_t s_dispatch_route_port;
static TaskHandle_t s_dispatch_route_task;

static SemaphoreHandle_t tx_mutex_get(void);

/* CRC16-CCITT: poly 0x1021, init 0xFFFF */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    while (len--) {
        crc ^= (uint16_t)*data++ << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000U)
                crc = (crc << 1) ^ 0x1021U;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

/* COBS encode: encode src[0..src_len-1] into dst; no 0x00 in encoded block.
 * Returns encoded length, or 0 if dst too small. Blocks of 254 non-zero bytes use code 255.
 */
static size_t cobs_encode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_cap)
{
    if (dst_cap < 2)
        return 0;
    size_t out = 0;
    size_t run_start = 0;
    while (run_start <= src_len) {
        if (run_start == src_len) {
            if (out + 1 > dst_cap)
                return 0;
            dst[out++] = 0x00;
            return out;
        }
        size_t block_len = 0;
        while (run_start + block_len < src_len && src[run_start + block_len] != 0 && block_len < 254U)
            block_len++;
        if (out + 1 + block_len > dst_cap)
            return 0;
        dst[out++] = (uint8_t)(block_len + 1);
        for (size_t k = 0; k < block_len; k++)
            dst[out++] = src[run_start + k];
        run_start += block_len;
        if (run_start < src_len && src[run_start] == 0)
            run_start++;
    }
    if (out + 1 > dst_cap)
        return 0;
    dst[out++] = 0x00;
    return out;
}

/* COBS decode: decode src into dst. Stops at first 0x00 if present; otherwise consumes all src.
 * Caller passes the block without the trailing 0x00, so we return out when input is consumed. */
static size_t cobs_decode(const uint8_t *src, size_t src_len, uint8_t *dst, size_t dst_cap)
{
    size_t out = 0;
    for (size_t i = 0; i < src_len; ) {
        uint8_t code = src[i++];
        if (code == 0x00)
            return out;
        if (code > 1) {
            size_t copy = (size_t)(code - 1);
            if (i + copy > src_len || out + copy > dst_cap)
                return 0;
            for (size_t k = 0; k < copy; k++)
                dst[out++] = src[i++];
        }
        /* Stuffed zero: output one 0 after block, except after last (next byte would be delimiter). */
        if (code != 255 && out < dst_cap) {
            if (code == 1)
                dst[out++] = 0;
            else if (i < src_len && src[i] != 0x00)
                dst[out++] = 0;
        }
    }
    return out;
}

void protocol_set_endpoint(uint8_t port) {
    if (port < UART_COUNT)
        s_active_proto_port = (uart_id_t)port;
}

uint8_t protocol_get_endpoint(void)
{
    return (uint8_t)s_active_proto_port;
}

// getting to here
/* Update the signature in uart_framing.h as well */
void protocol_send_frame(uint16_t msg_id, const uint8_t *payload, size_t len)
{
    if (payload == NULL || len > PROTOCOL_MAX_PAYLOAD)
        return;

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
    
    size_t raw_len = 7 + len;

    uint8_t cobs_buf[UART_TX_MAX_LEN];
    size_t cobs_len = cobs_encode(raw, raw_len, cobs_buf, sizeof(cobs_buf));
    
    if (cobs_len == 0)
        return;

    SemaphoreHandle_t tx_mutex = tx_mutex_get();
    if (tx_mutex == NULL)
        return;

    if (xSemaphoreTake(tx_mutex, portMAX_DELAY) != pdTRUE)
        return;

    uart_id_t tx_port = s_active_proto_port;
    if (s_dispatch_route_active && (xTaskGetCurrentTaskHandle() == s_dispatch_route_task)) {
        tx_port = s_dispatch_route_port;
    }
    uart_driver_transmit(tx_port, cobs_buf, cobs_len);

    (void)xSemaphoreGive(tx_mutex);
}

/* Streaming decoder state */
#define DECODE_RAW_CAP (PROTOCOL_HEADER_LEN + PROTOCOL_MAX_PAYLOAD)
typedef struct {
    uint8_t cobs_buf[UART_TX_MAX_LEN];
    size_t cobs_len;
    uint8_t raw_buf[DECODE_RAW_CAP];
} protocol_decoder_state_t;

static protocol_decoder_state_t s_decoders[UART_COUNT];
static StaticSemaphore_t s_tx_mutex_buf;
static SemaphoreHandle_t s_tx_mutex;
static StaticSemaphore_t s_dispatch_mutex_buf;
static SemaphoreHandle_t s_dispatch_mutex;

static SemaphoreHandle_t tx_mutex_get(void)
{
    if (s_tx_mutex == NULL) {
        s_tx_mutex = xSemaphoreCreateMutexStatic(&s_tx_mutex_buf);
    }
    return s_tx_mutex;
}

static SemaphoreHandle_t dispatch_mutex_get(void)
{
    if (s_dispatch_mutex == NULL) {
        s_dispatch_mutex = xSemaphoreCreateMutexStatic(&s_dispatch_mutex_buf);
    }
    return s_dispatch_mutex;
}

void uart_framing_feed(uint8_t port, const uint8_t *buf, size_t len)
{
    if (port >= UART_COUNT || buf == NULL || len == 0U) {
        return;
    }

    protocol_decoder_state_t *decoder = &s_decoders[port];

    for (size_t i = 0; i < len; i++) {
        uint8_t b = buf[i];
        if (b == 0x00) {
            /* End of COBS block */
            if (decoder->cobs_len > 0) {
                size_t raw_len = cobs_decode(decoder->cobs_buf, decoder->cobs_len,
                                            decoder->raw_buf, sizeof(decoder->raw_buf));
                decoder->cobs_len = 0;
                if (raw_len >= PROTOCOL_HEADER_LEN) {
                    uint16_t payload_len = (uint16_t)decoder->raw_buf[3]
                        | ((uint16_t)decoder->raw_buf[4] << 8);
                    if (payload_len <= PROTOCOL_MAX_PAYLOAD && raw_len == (size_t)(7U + payload_len)) {
                        uint16_t crc = (uint16_t)decoder->raw_buf[5 + payload_len]
                            | ((uint16_t)decoder->raw_buf[6 + payload_len] << 8);
                        if (crc == crc16_ccitt(decoder->raw_buf, 5 + payload_len)) {
                            uint16_t msg_id = (uint16_t)decoder->raw_buf[1]
                                | ((uint16_t)decoder->raw_buf[2] << 8);
                            const uint8_t *payload = decoder->raw_buf + 5;
                            // NEW: Temporarily set the response endpoint to the incoming port
                            SemaphoreHandle_t dispatch_mutex = dispatch_mutex_get();
                            if (dispatch_mutex != NULL && xSemaphoreTake(dispatch_mutex, portMAX_DELAY) == pdTRUE) {
                                protocol_set_endpoint(port);
                                s_dispatch_route_active = true;
                                s_dispatch_route_port = (uart_id_t)port;
                                s_dispatch_route_task = xTaskGetCurrentTaskHandle();
                                protocol_dispatch(msg_id, payload, payload_len);
                                s_dispatch_route_active = false;
                                s_dispatch_route_task = NULL;
                                (void)xSemaphoreGive(dispatch_mutex);
                            }
                        }

                    }
                }
            }
            continue;
        }
        if (decoder->cobs_len < sizeof(decoder->cobs_buf))
            decoder->cobs_buf[decoder->cobs_len++] = b;
        else
            decoder->cobs_len = 0; /* overflow, reset */
    }

}
