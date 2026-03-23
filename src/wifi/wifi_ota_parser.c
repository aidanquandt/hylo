#include "wifi_ota_parser.h"
#include "uart_framing.h"
#include "timer_driver.h"
#include <string.h>

/* Independent decoder for the WiFi/ESP8266 UART */
static uart_decoder_t wifi_decoder = {0};
/* Track timestamp of last successful WiFi decode for watchdog */
static volatile uint32_t s_last_wifi_decode_time = 0;

void ota_parser_init(void) {
    memset(&wifi_decoder, 0, sizeof(wifi_decoder));
    wifi_decoder.source = PROTOCOL_UART_DEST_WIFI;
    s_last_wifi_decode_time = 0;
}

uint32_t ota_parser_get_last_decode_time(void) {
    return s_last_wifi_decode_time;
}

void ota_parser_set_last_decode_time(uint32_t timestamp_ms) {
    s_last_wifi_decode_time = timestamp_ms;
}

void ota_parser_process_bytes(const uint8_t* data, size_t len) {
    if (data != NULL && len > 0) {
        /* Update last decode time whenever we receive data on WiFi */
        s_last_wifi_decode_time = timer_driver_get_time_ms();
        /* Feed the wifi's private decoder context */
        uart_framing_feed(&wifi_decoder, data, len);
    }
}

void ota_parser_process_byte(uint8_t byte) {
    ota_parser_process_bytes(&byte, 1);
}