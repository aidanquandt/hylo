#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef void (*uart_cmd_callback_t)(const char* cmd, uint16_t length);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
bool uart_manager_transmit(const uint8_t* data, size_t length);
bool uart_manager_print(const char* format, ...) __attribute__((format(printf, 1, 2)));
uint32_t uart_manager_get_queue_count(void);
uint32_t uart_manager_get_dropped_count(void);
uint32_t uart_manager_get_tx_errors(void);
void uart_manager_register_cmd_callback(uart_cmd_callback_t callback);
uint32_t uart_manager_get_rx_count(void);
uint32_t uart_manager_get_rx_overruns(void);
