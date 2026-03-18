/*---------------------------------------------------------------------------
 * @file    transport.h
 * @brief   Transport abstraction for protocol: UART or Wi-Fi
 *---------------------------------------------------------------------------*/
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    TRANSPORT_UART = 0,
    TRANSPORT_WIFI = 1
} transport_mode_t;

void transport_init(void);
void transport_set_mode(transport_mode_t mode);
transport_mode_t transport_get_mode(void);

/** Send framed bytes. Returns false if not ready (e.g. WiFi mode but disconnected). */
bool transport_send(const uint8_t *buf, size_t len);

/** Feed received bytes to framing decoder; on complete frame calls protocol_dispatch. */
void transport_feed(const uint8_t *buf, size_t len);

/** True when WiFi is connected and in ACTIVE state. */
bool transport_wifi_ready(void);
