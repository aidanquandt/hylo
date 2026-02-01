/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "common.h"
#include "main.h"
#include "task.h"
#include "usart.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    PLATFORM_UART_SUCCESS = 0,
    PLATFORM_UART_ERROR,
    PLATFORM_UART_TIMEOUT,
    PLATFORM_UART_BUSY
} platform_uart_status_E;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
platform_uart_status_E platform_uart_transmit_blocking(const uint8_t* data, size_t length);
platform_uart_status_E platform_uart_transmit_dma(const uint8_t* data, size_t length);
void platform_uart_register_tx_task(TaskHandle_t task_handle);
platform_uart_status_E platform_uart_receive(UART_HandleTypeDef* huart, uint8_t* data,
                                             size_t length);
platform_uart_status_E platform_uart_start_rx_dma(uint8_t* buffer, uint16_t size);
uint16_t platform_uart_get_rx_dma_position(void);