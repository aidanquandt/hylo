/*---------------------------------------------------------------------------
 * @file    uart_manager.c
 * @brief   UART management module - handles async UART transmission task
 * @details Implements the application-layer UART task that processes queued
 *          messages and coordinates with the platform layer for HW access
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_manager.h"
#include "module.h"
#include "platform_uart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_QUEUE_SIZE          32U     // Max messages queued
#define UART_TX_MSG_MAX_LENGTH      256U    // Max bytes per message
#define UART_TASK_STACK_SIZE        512U    // Stack for TX task (in words)
#define UART_TASK_PRIORITY          osPriorityNormal
#define UART_PRINT_BUFFER_SIZE      256U    // Printf buffer size

/*---------------------------------------------------------------------------
 * Private Types
 *---------------------------------------------------------------------------*/

/**
 * @brief UART message structure for queue
 */
typedef struct {
    uint8_t data[UART_TX_MSG_MAX_LENGTH];  ///< Message payload
    uint16_t length;                        ///< Valid bytes in data
} uart_tx_message_t;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uart_manager_init(void);
STATIC void uart_manager_create_task(void);

extern const module_S uart_manager_module;
const module_S uart_manager_module = {
    .module_init = uart_manager_init,
    .module_create_task = uart_manager_create_task,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC osMessageQueueId_t uart_tx_queue = NULL;     ///< FreeRTOS message queue
STATIC osThreadId_t uart_tx_task_handle = NULL;     ///< TX task handle
STATIC volatile uint32_t dropped_messages = 0U;     ///< Count of dropped messages

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

STATIC void uart_tx_task(void *argument);
STATIC bool is_in_isr_context(void);
STATIC bool uart_manager_queue_message(const uint8_t *data, uint16_t length, bool from_isr, uint32_t timeout_ms);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Check if currently executing in interrupt context
 * @return true if in ISR, false if in task context
 */
STATIC bool is_in_isr_context(void)
{
    return (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U;
}

/**
 * @brief Dedicated UART transmit task (runs asynchronously)
 * @param argument Unused
 */
STATIC void uart_tx_task(void *argument)
{
    (void)argument;
    uart_tx_message_t msg;
    
    for (;;) {
        // Block waiting for message (suspended, 0% CPU usage)
        osStatus_t status = osMessageQueueGet(uart_tx_queue, 
                                              &msg, 
                                              NULL, 
                                              osWaitForever);
        
        if (status == osOK) {
            // Transmit via platform layer (this task blocks, not callers)
            platform_uart_transmit_blocking(msg.data, msg.length);
        }
    }
}

/**
 * @brief Initialize UART manager module (create queue only)
 * @details Queue must be created before task, so this runs in init phase
 */
STATIC void uart_manager_init(void)
{
    // Create message queue
    uart_tx_queue = osMessageQueueNew(UART_TX_QUEUE_SIZE, 
                                      sizeof(uart_tx_message_t), 
                                      NULL);
    if (uart_tx_queue == NULL) {
        // Fatal error - halt for debugging
        for(;;);
    }
    
    dropped_messages = 0U;
}

/**
 * @brief Create UART transmit task
 * @details Called during task creation phase, after scheduler starts
 */
STATIC void uart_manager_create_task(void)
{
    // Create dedicated transmit task
    const osThreadAttr_t task_attr = {
        .name = "UART_TX",
        .stack_size = UART_TASK_STACK_SIZE * sizeof(uint32_t),
        .priority = UART_TASK_PRIORITY,
    };
    
    uart_tx_task_handle = osThreadNew(uart_tx_task, NULL, &task_attr);
    if (uart_tx_task_handle == NULL) {
        // Fatal error - halt for debugging
        for(;;);
    }
}

/*---------------------------------------------------------------------------
 * Private Function Implementations (Internal Helpers)
 *---------------------------------------------------------------------------*/

/**
 * @brief Queue message for asynchronous transmission (private helper)
 * @param data Pointer to data buffer
 * @param length Number of bytes to transmit
 * @param from_isr True if called from ISR context
 * @param timeout_ms Timeout in milliseconds (0 = no wait)
 * @return true if queued successfully, false if queue full
 */
STATIC bool uart_manager_queue_message(const uint8_t *data, uint16_t length, bool from_isr, uint32_t timeout_ms)
{
    if (uart_tx_queue == NULL || data == NULL || length == 0U || length > UART_TX_MSG_MAX_LENGTH) {
        return false;
    }
    
    // Build message
    uart_tx_message_t msg;
    msg.length = length;
    memcpy(msg.data, data, length);
    
    // Queue message
    osStatus_t status;
    if (from_isr) {
        status = osMessageQueuePut(uart_tx_queue, &msg, 0U, 0U);
    } else {
        status = osMessageQueuePut(uart_tx_queue, &msg, 0U, timeout_ms);
    }
    
    if (status != osOK) {
        // Queue full - increment drop counter
        dropped_messages++;
        return false;
    }
    
    return true;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool uart_manager_transmit(const uint8_t *data, size_t length)
{
    // Validate parameters
    if (data == NULL || length == 0U || length > UART_TX_MSG_MAX_LENGTH) {
        return false;
    }
    
    // Detect ISR context and set appropriate timeout
    bool from_isr = is_in_isr_context();
    uint32_t timeout_ms = from_isr ? 0U : 10U;  // Brief wait in task context
    
    return uart_manager_queue_message(data, (uint16_t)length, from_isr, timeout_ms);
}

bool uart_manager_print(const char *format, ...)
{
    if (format == NULL) {
        return false;
    }
    
    // Format string into local buffer
    char buffer[UART_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    
    int length = vsnprintf(buffer, UART_PRINT_BUFFER_SIZE, format, args);
    
    va_end(args);
    
    // Check for formatting errors
    if (length < 0) {
        return false;
    }
    
    // Truncate if buffer was too small
    if (length >= UART_PRINT_BUFFER_SIZE) {
        length = UART_PRINT_BUFFER_SIZE - 1;
    }
    
    // Queue for asynchronous transmission
    return uart_manager_transmit((const uint8_t *)buffer, (size_t)length);
}

uint32_t uart_manager_get_queue_count(void)
{
    if (uart_tx_queue == NULL) {
        return 0U;
    }
    
    return osMessageQueueGetCount(uart_tx_queue);
}

uint32_t uart_manager_get_dropped_count(void)
{
    return dropped_messages;
}
