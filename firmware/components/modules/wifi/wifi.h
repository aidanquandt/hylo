/*---------------------------------------------------------------------------
 * @file    wifi.h
 * @brief   WiFi telemetry module for data transmission
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

// HAL callback hooks (call these from your global callbacks)
void ESP_RX_UARTEx_RXEventCallback(uint16_t size);
void ESP_RX_UART_ErrorCallback(void);