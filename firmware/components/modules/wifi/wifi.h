/*---------------------------------------------------------------------------
 * @file    wifi.h
 * @brief   ESP01 WiFi module interface
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Check if WiFi module is connected to TCP server
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Get number of successful transmissions
 * @return Count of successful WiFi transmissions
 */
uint32_t wifi_get_tx_count(void);

/**
 * @brief Get number of received responses from ESP
 * @return Count of responses received
 */
uint32_t wifi_get_rx_count(void);

/**
 * @brief Check if WiFi module is ready
 * @return true if initialized and ready, false otherwise
 */
bool wifi_is_ready(void);
