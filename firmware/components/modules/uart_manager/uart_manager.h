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
 * @note Thread-safe: Can be called from any task or ISR context
 * @note Message is queued and transmitted asynchronously by UART task
 */
bool uart_manager_transmit(const uint8_t *data, size_t length);

/**
 * @brief Print formatted string to UART (printf-style, thread-safe)
 * @param format Format string (printf-style)
 * @param ... Variable arguments
 * @return true if queued successfully, false on failure
 * @note Maximum formatted output length is 256 characters (truncated if longer)
 * @note Thread-safe: Can be called from any task or ISR context
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
 * @note Saturates at UINT32_MAX to prevent rollover
 */
uint32_t uart_manager_get_dropped_count(void);

/**
 * @brief Get total number of transmission errors
 * @return Cumulative count of platform-level TX errors since init
 */
uint32_t uart_manager_get_tx_errors(void);

/*---------------------------------------------------------------------------
 * Command Reception (RX) Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Command callback function type
 * @param cmd Null-terminated command string
 * @param length Length of command (excluding null terminator)
 */
typedef void (*uart_cmd_callback_t)(const char *cmd, uint16_t length);

/**
 * @brief Register callback for received commands
 * @param callback Function to call when complete command received (or NULL to unregister)
 * @note Callback is called from 10Hz periodic context (not ISR)
 * @note Commands are delimited by newline ('\n') or carriage return ('\r')
 * @note CR+LF pairs (\r\n) are treated as a single delimiter
 * @note Callback execution blocks RX processing - keep handlers short
 * 
 * @example
 * void my_cmd_handler(const char *cmd, uint16_t length) {
 *     if (strcmp(cmd, "hello") == 0) {
 *         uart_manager_print("Hello World!\\n");
 *     } else if (strcmp(cmd, "status") == 0) {
 *         uart_manager_print("System OK\\n");
 *     }
 * }
 * 
 * // In your init:
 * uart_manager_register_cmd_callback(my_cmd_handler);
 */
void uart_manager_register_cmd_callback(uart_cmd_callback_t callback);

/**
 * @brief Get number of commands received since init
 * @return Total commands processed
 */
uint32_t uart_manager_get_rx_count(void);

/**
 * @brief Get number of buffer overruns (commands too long)
 * @return Total commands discarded due to length exceeding buffer
 * @note Saturates at UINT32_MAX to prevent rollover
 */
uint32_t uart_manager_get_rx_overruns(void);
