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
#include "uart_cmd_router.h"
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

// RX DMA buffer settings
#define UART_RX_BUFFER_SIZE         256U    // DMA circular buffer size
#define UART_CMD_MAX_LENGTH         128U    // Max command length
#define UART_CMD_DELIMITER          '\n'    // Command delimiter (Enter key)
#define UART_ECHO_ENABLED           1       // Set to 0 to disable character echo

// Echo sequences
#define UART_ECHO_BACKSPACE_SEQ     "\b \b"    // Backspace sequence: BS + space + BS
#define UART_ECHO_BACKSPACE_LEN     3U
#define UART_ECHO_NEWLINE_SEQ       "\r\n"     // Newline sequence: CR + LF
#define UART_ECHO_NEWLINE_LEN       2U

// Queue timeout
#define UART_QUEUE_TIMEOUT_MS       10U     // Timeout for queue put from task context

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
STATIC void uart_manager_process_10Hz(void);

extern const module_S uart_manager_module;
const module_S uart_manager_module = {
    .module_name = "uart_manager",
    .module_init = uart_manager_init,
    .module_create_task = uart_manager_create_task,
    .module_process_10Hz = uart_manager_process_10Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC osMessageQueueId_t uart_tx_queue = NULL;     ///< FreeRTOS message queue
STATIC osThreadId_t uart_tx_task_handle = NULL;     ///< TX task handle
STATIC volatile uint32_t dropped_messages = 0U;     ///< Count of dropped messages
STATIC volatile uint32_t tx_errors = 0U;            ///< Count of transmission errors

// RX DMA buffer and state
STATIC uint8_t rx_dma_buffer[UART_RX_BUFFER_SIZE] __attribute__((section(".dma_buffer")));  ///< DMA circular buffer (D2 RAM for cache bypass)
STATIC uint8_t cmd_buffer[UART_CMD_MAX_LENGTH];     ///< Command processing buffer
STATIC uint16_t cmd_length = 0U;                    ///< Current command length being built
STATIC uint16_t last_checked_pos = 0U;              ///< Last position checked in DMA buffer
STATIC uart_cmd_callback_t cmd_callback = NULL;     ///< User command callback
STATIC uint32_t rx_commands_received = 0U;          ///< Total commands received
STATIC uint32_t rx_buffer_overruns = 0U;            ///< Total buffer overruns
STATIC bool startup_message_sent = false;           ///< Flag to send welcome message once
STATIC volatile bool command_in_progress = false;   ///< Flag to prevent re-entrant command processing
STATIC bool last_was_cr = false;                    ///< Flag to handle \r\n pairs as single delimiter

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

STATIC void uart_tx_task(void *argument);
STATIC bool is_in_isr_context(void);
STATIC bool uart_manager_queue_message(const uint8_t *data, uint16_t length, bool from_isr, uint32_t timeout_ms);
STATIC void uart_manager_process_rx_buffer(void);
STATIC void uart_manager_handle_command(void);
STATIC uint16_t uart_manager_trim_command(uint16_t length);
STATIC void uart_manager_default_cmd_handler(const char *cmd, uint16_t length);

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
        
        if (status == osOK && msg.length > 0U && msg.length <= UART_TX_MSG_MAX_LENGTH) {
            // Transmit via platform layer (this task blocks, not callers)
            platform_uart_status_E tx_status = platform_uart_transmit_blocking(msg.data, msg.length);
            
            // Track transmission errors
            if (tx_status != PLATFORM_UART_SUCCESS) {
                tx_errors++;
            }
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
    
    // Initialize RX state
    cmd_length = 0U;
    last_checked_pos = 0U;
    rx_commands_received = 0U;
    rx_buffer_overruns = 0U;
    last_was_cr = false;
    tx_errors = 0U;
    
    // Initialize command router
    uart_cmd_router_init();
    
    // Register command router as default callback
    uart_manager_register_cmd_callback(uart_manager_default_cmd_handler);
    
    // Start DMA reception via platform layer (polling mode - no interrupts)
    platform_uart_start_rx_dma(rx_dma_buffer, UART_RX_BUFFER_SIZE);
}

/**
 * @brief Default command handler - routes to command router
 */
STATIC void uart_manager_default_cmd_handler(const char *cmd, uint16_t length)
{
    (void)length;  // Unused - cmd is null-terminated
    uart_cmd_router_dispatch(cmd);
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
        // Queue full - increment drop counter atomically
        // Note: On Cortex-M, 32-bit aligned reads/writes are atomic
        // For extra safety, could use __atomic_fetch_add or disable interrupts
        uint32_t current;
        do {
            current = dropped_messages;
        } while (current == UINT32_MAX);  // Saturate at max
        dropped_messages = current + 1U;
        return false;
    }
    
    return true;
}

/**
 * @brief Trim leading and trailing whitespace from command buffer
 * @details Modifies cmd_buffer in-place and returns new length
 * @return New command length after trimming
 */
STATIC uint16_t uart_manager_trim_command(uint16_t length)
{
    if (length == 0) {
        return 0;
    }
    
    // Trim leading whitespace
    uint16_t start = 0;
    while (start < length && (cmd_buffer[start] == ' ' || cmd_buffer[start] == '\t')) {
        start++;
    }
    
    // Trim trailing whitespace
    uint16_t end = length;
    while (end > start && (cmd_buffer[end - 1] == ' ' || cmd_buffer[end - 1] == '\t')) {
        end--;
    }
    
    // Calculate new length
    uint16_t new_length = end - start;
    
    // Shift command to beginning only if there's leading whitespace
    if (new_length > 0) {
        if (start > 0) {
            memmove(cmd_buffer, &cmd_buffer[start], new_length);
        }
        // Null terminate
        cmd_buffer[new_length] = '\0';
    } else {
        // Empty command
        cmd_buffer[0] = '\0';
    }
    
    return new_length;
}

/**
 * @brief Process completed command
 */
STATIC void uart_manager_handle_command(void)
{
    // Trim whitespace
    uint16_t trimmed_length = uart_manager_trim_command(cmd_length);
    
    // Skip if command is empty after trimming
    if (trimmed_length == 0) {
        cmd_length = 0;
        return;
    }
    
    // Increment counter
    rx_commands_received++;
    
    // Make a local copy of the command to prevent corruption during callback
    char cmd_copy[UART_CMD_MAX_LENGTH];
    memcpy(cmd_copy, cmd_buffer, trimmed_length + 1);  // +1 for null terminator
    
    // Reset command buffer for next command
    cmd_length = 0;
    
    // Set flag to prevent re-entrant processing
    command_in_progress = true;
    
    // Call user callback if registered (using local copy)
    if (cmd_callback != NULL) {
        cmd_callback((const char *)cmd_copy, trimmed_length);
    }
    
    // Clear flag after command completes
    command_in_progress = false;
}

/**
 * @brief Poll DMA buffer for new data and process commands
 * @details Checks for delimiter characters and extracts complete commands
 */
STATIC void uart_manager_process_rx_buffer(void)
{
    // Skip processing if a command is currently being executed
    // This prevents re-entrant calls that could corrupt the parsing state
    if (command_in_progress) {
        return;
    }
    
    // Get current DMA write position via platform layer
    uint16_t dma_write_pos = platform_uart_get_rx_dma_position();
    
    // Process all new characters
    while (last_checked_pos != dma_write_pos) {
        char c = (char)rx_dma_buffer[last_checked_pos];
        
        // Handle backspace/delete (ASCII 0x08 or 0x7F)
        if (c == 0x08 || c == 0x7F) {
            if (cmd_length > 0) {
                cmd_length--;  // Remove last character from command
#if UART_ECHO_ENABLED
                // Echo backspace sequence: backspace + space + backspace (erases character on screen)
                uart_manager_transmit((const uint8_t *)UART_ECHO_BACKSPACE_SEQ, UART_ECHO_BACKSPACE_LEN);
#endif
            }
            last_was_cr = false;  // Reset CR state on backspace
            // Move to next character in DMA buffer
            last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
            continue;
        }
        
        // Check for command delimiter (Enter key)
        // Handle both \r\n (Windows) and \n (Unix) as single delimiter
        if (c == UART_CMD_DELIMITER || c == '\r') {
            // Skip LF if previous character was CR (prevents double processing of \r\n)
            if (c == UART_CMD_DELIMITER && last_was_cr) {
                last_was_cr = false;
                last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
                continue;
            }
            
            last_was_cr = (c == '\r');
            
#if UART_ECHO_ENABLED
            // Echo newline for clean formatting
            uart_manager_transmit((const uint8_t *)UART_ECHO_NEWLINE_SEQ, UART_ECHO_NEWLINE_LEN);
#endif
            // Null-terminate and process command
            cmd_buffer[cmd_length] = '\0';
            uart_manager_handle_command();
            // cmd_length reset inside handler
        }
        else {
            last_was_cr = false;  // Reset CR state for regular characters
            
            // Regular character - add to command buffer
#if UART_ECHO_ENABLED
            // Echo character for local feedback
            uart_manager_transmit((const uint8_t *)&c, 1);
#endif
            
            // Add to command buffer if there's space
            if (cmd_length < UART_CMD_MAX_LENGTH - 1) {
                cmd_buffer[cmd_length++] = (uint8_t)c;
            } else {
                // Buffer full - increment overrun counter (saturate at max)
                if (rx_buffer_overruns < UINT32_MAX) {
                    rx_buffer_overruns++;
                }
            }
        }
        
        // Move to next character (circular)
        last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
    }
}

/**
 * @brief Periodic RX processing at 10Hz
 * @details Polls DMA buffer for incoming commands
 */
STATIC void uart_manager_process_10Hz(void)
{
    // Send welcome message on first call (after TX task is running)
    if (!startup_message_sent) {
        startup_message_sent = true;
        uart_manager_print("\r\n=== UART Command System Ready ===\r\n");
        uart_manager_print("Type 'help' for available commands\r\n\r\n");
    }
    
    uart_manager_process_rx_buffer();
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
    uint32_t timeout_ms = from_isr ? 0U : UART_QUEUE_TIMEOUT_MS;  // Brief wait in task context
    
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

void uart_manager_register_cmd_callback(uart_cmd_callback_t callback)
{
    cmd_callback = callback;
}

uint32_t uart_manager_get_rx_count(void)
{
    return rx_commands_received;
}

uint32_t uart_manager_get_rx_overruns(void)
{
    return rx_buffer_overruns;
}

uint32_t uart_manager_get_tx_errors(void)
{
    return tx_errors;
}
