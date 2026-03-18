/*---------------------------------------------------------------------------
 * @file    transport.c
 * @brief   Transport abstraction: route protocol bytes over UART or Wi-Fi
 *---------------------------------------------------------------------------*/
#include "transport.h"
#include "uart_driver.h"
#include "uart_framing.h"
#include "wifi.h"
#include <stddef.h>
#include <string.h>

static transport_mode_t s_mode = TRANSPORT_UART;

void transport_init(void)
{
    s_mode = TRANSPORT_UART;
}

void transport_set_mode(transport_mode_t mode)
{
    s_mode = mode;
}

transport_mode_t transport_get_mode(void)
{
    return s_mode;
}

bool transport_send(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len == 0)
        return false;

    if (s_mode == TRANSPORT_UART)
    {
        uart_driver_transmit(UART_CONSOLE, buf, len);
        return true;
    }

    if (s_mode == TRANSPORT_WIFI)
    {
        if (!transport_wifi_ready())
            return false;
        wifi_send_protocol_bytes(buf, len);
        return true;
    }

    return false;
}

void transport_feed(const uint8_t *buf, size_t len)
{
    uart_framing_feed(buf, len);
}

bool transport_wifi_ready(void)
{
    return wifi_telemetry_is_ready();
}
