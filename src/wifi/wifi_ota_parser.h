#pragma once

#include "common.h"

void ota_parser_init(void);
void ota_parser_process_byte(uint8_t byte); // feed each byte from WiFi RX