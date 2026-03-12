/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "common.h"
#include "task.h"
#include <stdbool.h>
#include <stddef.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UART_DRIVER_SUCCESS = 0,
    UART_DRIVER_ERROR,
    UART_DRIVER_TIMEOUT,
    UART_DRIVER_BUSY
} uart_driver_status_E;

/** Callback for complete RX line (driver delivers one line at a time) */
typedef void (*uart_driver_rx_line_callback_t)(const char* line, uint16_t length);

/*---------------------------------------------------------------------------
 * Public Function Prototypes - Console UART (queue + task in driver)
 *---------------------------------------------------------------------------*/
/** Initialize console UART: create TX queue/task and start RX DMA. Call once before using send/print. */
void uart_driver_console_init(void);
/** Enqueue data for TX (non-blocking). Returns true if queued. */
bool uart_driver_send(const uint8_t* data, size_t length);
/** Format and enqueue for TX. Returns true if queued. */
bool uart_driver_print(const char* format, ...) __attribute__((format(printf, 1, 2)));
/** Register callback invoked when a complete line is received (after \\n or \\r). */
void uart_driver_register_rx_line_callback(uart_driver_rx_line_callback_t callback);
/** Poll RX DMA and assemble lines; call from app task (e.g. 100 Hz). Delivers lines via callback. */
void uart_driver_poll_rx(void);
uint32_t uart_driver_get_tx_queue_count(void);
uint32_t uart_driver_get_tx_dropped_count(void);
uint32_t uart_driver_get_tx_errors(void);
uint32_t uart_driver_get_rx_overruns(void);

/* Legacy / low-level (still available for direct use) */
uart_driver_status_E uart_driver_transmit_blocking(const uint8_t* data, size_t length);
uart_driver_status_E uart_driver_transmit_dma(const uint8_t* data, size_t length);
uart_driver_status_E uart_driver_start_rx_dma(uint8_t* buffer, uint16_t size);
uint16_t uart_driver_get_rx_dma_position(void);

/*---------------------------------------------------------------------------
 * WIFI UART (separate instance)
 *---------------------------------------------------------------------------*/
uart_driver_status_E uart_driver_wifi_transmit_blocking(const uint8_t* data, size_t length);
void uart_driver_wifi_rx_init(void);
