#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

/*---------------------------------------------------------------------------
 * Public function prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Transmit data over UART (thread-safe, non-blocking)
 * @param data Pointer to data buffer
 * @param length Number of bytes to transmit (max 256)
 * @return true if queued successfully, false if queue full or invalid params
 * @note Safe to call from ISR context
 * @note Message is queued and transmitted asynchronously by UART task
 */
bool uart_manager_transmit(const uint8_t *data, size_t length);

/**
 * @brief Print formatted string to UART (printf-style, thread-safe)
 * @param format Format string (printf-style)
 * @param ... Variable arguments
 * @return true if queued successfully, false on failure
 * @note Maximum formatted output length is 256 characters (truncated if longer)
 * @note Safe to call from ISR context
 */
bool uart_manager_print(const char *format, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Get number of messages waiting in UART transmit queue
 * @return Number of pending messages (0 = queue empty)
 * @note Useful for monitoring queue health and detecting logging storms
 */
uint32_t uart_manager_get_queue_count(void);

/**
 * @brief Get total number of messages dropped due to queue full
 * @return Cumulative count of dropped messages since init
 */
uint32_t uart_manager_get_dropped_count(void);
