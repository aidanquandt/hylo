/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "imu.h"
#include "module.h"
#include "platform_os.h"
#include "platform_system.h"
#include "platform_uart.h"
#include "queue.h"
#include "task.h"
#include "task_config.h"
#include "uart_cmd_router.h"
#include "uwb.h"
#include "watchdog.h"


/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_TX_QUEUE_SIZE 32U
#define UART_TX_MSG_MAX_LENGTH 256U
#define UART_TASK_STACK_SIZE 512U
#define UART_RX_TASK_STACK_SIZE 256U
#define UART_PRINT_BUFFER_SIZE 256U
#define UART_RX_BUFFER_SIZE 256U
#define UART_CMD_MAX_LENGTH 128U
#define UART_CMD_DELIMITER '\n'
#define UART_ECHO_ENABLED 1
#define UART_ECHO_BACKSPACE_SEQ "\b \b"
#define UART_ECHO_BACKSPACE_LEN 3U
#define UART_ECHO_NEWLINE_SEQ "\r\n"
#define UART_ECHO_NEWLINE_LEN 2U
#define UART_QUEUE_TIMEOUT_MS 10U

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t data[UART_TX_MSG_MAX_LENGTH];
    uint16_t length;
} uart_tx_message_t;

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uart_manager_init(void);
STATIC void uart_manager_create_tasks(void);

extern const module_S uart_manager_module;

const module_S uart_manager_module = {
    .module_name         = "uart_manager",
    .module_init         = uart_manager_init,
    .module_create_tasks = uart_manager_create_tasks,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC QueueHandle_t uart_tx_queue        = NULL;
STATIC TaskHandle_t uart_tx_task_handle   = NULL;
STATIC volatile uint32_t dropped_messages = 0U;
STATIC volatile uint32_t tx_errors        = 0U;
STATIC uint8_t rx_dma_buffer[UART_RX_BUFFER_SIZE] __attribute__((section(".dma_buffer")));
STATIC uint8_t tx_dma_buffer[UART_TX_MSG_MAX_LENGTH] __attribute__((section(".dma_buffer")));
STATIC uint8_t cmd_buffer[UART_CMD_MAX_LENGTH];
STATIC uint16_t cmd_length               = 0U;
STATIC uint16_t last_checked_pos         = 0U;
STATIC uart_cmd_callback_t cmd_callback  = NULL;
STATIC uint32_t rx_commands_received     = 0U;
STATIC uint32_t rx_buffer_overruns       = 0U;
STATIC volatile bool command_in_progress = false;
STATIC bool last_was_cr                  = false;

// Recursion guard: prevent error_handler from using UART if UART is calling error_handler
STATIC volatile bool in_error_handler_call = false;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void uart_tx_task(void* argument);
STATIC void uart_rx_task(void* argument);
STATIC bool is_in_isr_context(void);
STATIC bool uart_manager_queue_message(const uint8_t* data, uint16_t length, bool from_isr,
                                       uint32_t timeout_ms);
STATIC void uart_manager_process_rx_buffer(void);
STATIC void uart_manager_handle_command(void);
STATIC uint16_t uart_manager_trim_command(uint16_t length);
STATIC void uart_manager_default_cmd_handler(const char* cmd, uint16_t length);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC bool is_in_isr_context(void)
{
    return (xPortIsInsideInterrupt() == pdTRUE);
}

STATIC void uart_tx_task(void* argument)
{
    (void)argument;
    uart_tx_message_t msg;

    // Register this task for TX complete notifications
    platform_uart_register_tx_task(xTaskGetCurrentTaskHandle());

    for (;;)
    {
        if (xQueueReceive(uart_tx_queue, &msg, portMAX_DELAY) == pdPASS)
        {
            if (msg.length > 0U && msg.length <= UART_TX_MSG_MAX_LENGTH)
            {
                // Copy message to DMA-accessible buffer (DMA cannot access DTCM RAM)
                memcpy(tx_dma_buffer, msg.data, msg.length);

                // Start DMA transmission (non-blocking)
                platform_uart_status_E tx_status =
                    platform_uart_transmit_dma(tx_dma_buffer, msg.length);

                if (tx_status == PLATFORM_UART_SUCCESS)
                {
                    // Wait for DMA transmission to complete (notification from ISR)
                    // Timeout after 100ms in case something goes wrong
                    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

                    if (notification == 0)
                    {
                        // Timeout - DMA didn't signal completion
                        // This indicates a potential hardware issue
                        tx_errors++;
                    }
                }
                else if (tx_status == PLATFORM_UART_BUSY)
                {
                    // Previous transmission still in progress
                    // This shouldn't happen with proper flow control
                    tx_errors++;
                }
                else
                {
                    // Other error (invalid parameters, etc.)
                    tx_errors++;
                }
            }
        }
    }
}

STATIC void uart_manager_init(void)
{
    // Create message queue
    uart_tx_queue = xQueueCreate(UART_TX_QUEUE_SIZE, sizeof(uart_tx_message_t));
    if (uart_tx_queue == NULL)
    {
        in_error_handler_call = true;
        error_handler_fatal("uart_manager", "Failed to create TX queue");
        in_error_handler_call = false;
    }

    dropped_messages = 0U;

    // Initialize RX state
    cmd_length           = 0U;
    last_checked_pos     = 0U;
    rx_commands_received = 0U;
    rx_buffer_overruns   = 0U;
    last_was_cr          = false;
    tx_errors            = 0U;

    // Initialize command router
    uart_cmd_router_init();

    // Register command router as default callback
    uart_manager_register_cmd_callback(uart_manager_default_cmd_handler);

    // Start DMA reception via platform layer (polling mode - no interrupts)
    platform_uart_start_rx_dma(rx_dma_buffer, UART_RX_BUFFER_SIZE);
}

STATIC void uart_manager_default_cmd_handler(const char* cmd, uint16_t length)
{
    (void)length; // Unused - cmd is null-terminated

    // Intercept 'system_reset' command and perform MCU reset directly
    if (strcmp(cmd, "system_reset") == 0)
    {
        uart_manager_print("\r\nSystem Resetting...\r\n");

        // Perform soft resets on peripherals before MCU reset
        // Use module-level functions which handle device state checking
        if (uwb_soft_reset())
        {
            uart_manager_print("UWB soft reset completed\r\n");
        }

        if (imu_soft_reset())
        {
            uart_manager_print("IMU soft reset completed\r\n");
        }

        // Small delay to allow reset messages to be transmitted
        vTaskDelay(pdMS_TO_TICKS(10));

        uart_manager_print("Performing MCU reset... \r\n");
        platform_system_reset();
    }
    else
    {
        // Otherwise, route to command router
        uart_cmd_router_dispatch(cmd);
    }
}

STATIC void uart_manager_create_tasks(void)
{
    // Create dedicated transmit task
    BaseType_t task_result = xTaskCreate(uart_tx_task, "uart_tx", UART_TASK_STACK_SIZE, NULL,
                                         TASK_PRIORITY_UART_TX, &uart_tx_task_handle);
    if (task_result != pdPASS)
    {
        in_error_handler_call = true;
        error_handler_fatal("uart_manager", "Failed to create UART TX task");
        in_error_handler_call = false;
    }

    // Create RX processing task (polls DMA buffer at 100Hz)
    task_result = xTaskCreate(uart_rx_task, "uart_rx", UART_RX_TASK_STACK_SIZE, NULL,
                              TASK_PRIORITY_UART_TX, NULL);
    if (task_result != pdPASS)
    {
        in_error_handler_call = true;
        error_handler_fatal("uart_manager", "Failed to create UART RX task");
        in_error_handler_call = false;
    }
}

STATIC bool uart_manager_queue_message(const uint8_t* data, uint16_t length, bool from_isr,
                                       uint32_t timeout_ms)
{
    // Defensive check: If TX task not created yet, silently drop the message
    // This protects against logging during module_init phase (before module_create_task)
    if (uart_tx_task_handle == NULL)
    {
        if (!from_isr && dropped_messages < UINT32_MAX)
        {
            dropped_messages++;
        }
        return false;
    }

    if (uart_tx_queue == NULL || data == NULL || length == 0U || length > UART_TX_MSG_MAX_LENGTH)
    {
        return false;
    }

    // Build message
    uart_tx_message_t msg;
    msg.length = length;
    memcpy(msg.data, data, length);

    // Queue message
    BaseType_t status;
    if (from_isr)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = xQueueSendFromISR(uart_tx_queue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        status = xQueueSend(uart_tx_queue, &msg, pdMS_TO_TICKS(timeout_ms));
    }

    if (status != pdPASS)
    {
        // Queue full - increment drop counter atomically
        // Note: On Cortex-M, 32-bit aligned reads/writes are atomic
        // For extra safety, could use __atomic_fetch_add or disable interrupts
        uint32_t current;
        do
        {
            current = dropped_messages;
        } while (current == UINT32_MAX); // Saturate at max
        dropped_messages = current + 1U;
        return false;
    }

    return true;
}

STATIC uint16_t uart_manager_trim_command(uint16_t length)
{
    if (length == 0)
    {
        return 0;
    }

    // Trim leading whitespace
    uint16_t start = 0;
    while (start < length && (cmd_buffer[start] == ' ' || cmd_buffer[start] == '\t'))
    {
        start++;
    }

    // Trim trailing whitespace
    uint16_t end = length;
    while (end > start && (cmd_buffer[end - 1] == ' ' || cmd_buffer[end - 1] == '\t'))
    {
        end--;
    }

    // Calculate new length
    uint16_t new_length = end - start;

    // Shift command to beginning only if there's leading whitespace
    if (new_length > 0)
    {
        if (start > 0)
        {
            memmove(cmd_buffer, &cmd_buffer[start], new_length);
        }
        // Null terminate
        cmd_buffer[new_length] = '\0';
    }
    else
    {
        // Empty command
        cmd_buffer[0] = '\0';
    }

    return new_length;
}

STATIC void uart_manager_handle_command(void)
{
    // Trim whitespace
    uint16_t trimmed_length = uart_manager_trim_command(cmd_length);

    // Skip if command is empty after trimming
    if (trimmed_length == 0)
    {
        cmd_length = 0;
        return;
    }

    // Increment counter
    rx_commands_received++;

    // Make a local copy of the command to prevent corruption during callback
    char cmd_copy[UART_CMD_MAX_LENGTH];
    memcpy(cmd_copy, cmd_buffer, trimmed_length + 1); // +1 for null terminator

    // Reset command buffer for next command
    cmd_length = 0;

    // Set flag to prevent re-entrant processing
    command_in_progress = true;

    // Call user callback if registered (using local copy)
    if (cmd_callback != NULL)
    {
        cmd_callback((const char*)cmd_copy, trimmed_length);
    }

    // Clear flag after command completes
    command_in_progress = false;
}

STATIC void uart_manager_process_rx_buffer(void)
{
    // Skip processing if a command is currently being executed
    // This prevents re-entrant calls that could corrupt the parsing state
    if (command_in_progress)
    {
        return;
    }

    // Get current DMA write position via platform layer
    uint16_t dma_write_pos = platform_uart_get_rx_dma_position();

    // Process all new characters
    while (last_checked_pos != dma_write_pos)
    {
        char c = (char)rx_dma_buffer[last_checked_pos];

        // Handle backspace/delete (ASCII 0x08 or 0x7F)
        if (c == 0x08 || c == 0x7F)
        {
            if (cmd_length > 0)
            {
                cmd_length--; // Remove last character from command
#if UART_ECHO_ENABLED
                // Echo backspace sequence: backspace + space + backspace (erases character on
                // screen)
                uart_manager_transmit((const uint8_t*)UART_ECHO_BACKSPACE_SEQ,
                                      UART_ECHO_BACKSPACE_LEN);
#endif
            }
            last_was_cr = false; // Reset CR state on backspace
            // Move to next character in DMA buffer
            last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
            continue;
        }

        // Check for command delimiter (Enter key)
        // Handle both \r\n (Windows) and \n (Unix) as single delimiter
        if (c == UART_CMD_DELIMITER || c == '\r')
        {
            // Skip LF if previous character was CR (prevents double processing of \r\n)
            if (c == UART_CMD_DELIMITER && last_was_cr)
            {
                last_was_cr      = false;
                last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
                continue;
            }

            last_was_cr = (c == '\r');

#if UART_ECHO_ENABLED
            // Echo newline for clean formatting
            uart_manager_transmit((const uint8_t*)UART_ECHO_NEWLINE_SEQ, UART_ECHO_NEWLINE_LEN);
#endif
            // Null-terminate and process command
            cmd_buffer[cmd_length] = '\0';
            uart_manager_handle_command();
            // cmd_length reset inside handler
        }
        else
        {
            last_was_cr = false; // Reset CR state for regular characters

            // Regular character - add to command buffer
#if UART_ECHO_ENABLED
            // Echo character for local feedback
            uart_manager_transmit((const uint8_t*)&c, 1);
#endif

            // Add to command buffer if there's space
            if (cmd_length < UART_CMD_MAX_LENGTH - 1)
            {
                cmd_buffer[cmd_length++] = (uint8_t)c;
            }
            else
            {
                // Buffer full - increment overrun counter (saturate at max)
                if (rx_buffer_overruns < UINT32_MAX)
                {
                    rx_buffer_overruns++;
                }
            }
        }

        // Move to next character (circular)
        last_checked_pos = (last_checked_pos + 1) % UART_RX_BUFFER_SIZE;
    }
}

STATIC void uart_rx_task(void* argument)
{
    (void)argument;

    TickType_t lastWake     = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10); // 100Hz = 10ms

    watchdog_register_task(50); // Expect heartbeat every 50ms

    for (;;)
    {
        uart_manager_process_rx_buffer();

        watchdog_heartbeat();
        vTaskDelayUntil(&lastWake, period);
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool uart_manager_transmit(const uint8_t* data, size_t length)
{
    // Validate parameters
    if (data == NULL || length == 0U || length > UART_TX_MSG_MAX_LENGTH)
    {
        return false;
    }

    // Detect ISR context and set appropriate timeout
    bool from_isr       = is_in_isr_context();
    uint32_t timeout_ms = from_isr ? 0U : UART_QUEUE_TIMEOUT_MS; // Brief wait in task context

    return uart_manager_queue_message(data, (uint16_t)length, from_isr, timeout_ms);
}

bool uart_manager_print(const char* format, ...)
{
    if (format == NULL)
    {
        return false;
    }

    // Prevent recursion: if error_handler calls us while we're calling error_handler, drop it
    if (in_error_handler_call)
    {
        return false;
    }

    // Format string into local buffer
    char buffer[UART_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);

    int length = vsnprintf(buffer, UART_PRINT_BUFFER_SIZE, format, args);

    va_end(args);

    // Check for formatting errors
    if (length < 0)
    {
        return false;
    }

    // Truncate if buffer was too small
    if (length >= (int)UART_PRINT_BUFFER_SIZE)
    {
        length = (int)UART_PRINT_BUFFER_SIZE - 1;
    }

    // Queue for asynchronous transmission
    return uart_manager_transmit((const uint8_t*)buffer, (size_t)length);
}

uint32_t uart_manager_get_queue_count(void)
{
    if (uart_tx_queue == NULL)
    {
        return 0U;
    }

    return (uint32_t)uxQueueMessagesWaiting(uart_tx_queue);
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
