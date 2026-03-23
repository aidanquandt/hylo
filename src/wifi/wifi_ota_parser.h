#pragma once

#include "common.h"

void ota_parser_init(void);
void ota_parser_process_byte(uint8_t byte);
void ota_parser_process_bytes(const uint8_t* data, size_t len);

/* Watchdog support: track when WiFi last successfully decoded a message */
uint32_t ota_parser_get_last_decode_time(void);
void ota_parser_set_last_decode_time(uint32_t timestamp_ms);