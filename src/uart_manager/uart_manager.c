/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_manager.h"
#include "FreeRTOS.h"
#include "common.h"
#include "error_handler.h"
#include "imu.h"
#include "module.h"
#include "os_driver.h"
#include "system_driver.h"
#include "uart_driver.h"
#include "task.h"
#include "uart_cmd_router.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "uwb.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UART_CMD_MAX_LENGTH 128U
#define UART_PRINT_BUFFER_SIZE 256U
#define UART_TX_MSG_MAX_LENGTH 256U
#define UART_QUEUE_TIMEOUT_MS 10U

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void uart_manager_init(void);
STATIC void uart_manager_create_task(void);
STATIC void uart_manager_process_10Hz(void);
STATIC void uart_manager_process_100Hz(void);

extern const module_S uart_manager_module;

const module_S uart_manager_module = {
    .module_name          = "uart_manager",
    .module_init          = uart_manager_init,
    .module_create_task   = uart_manager_create_task,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = uart_manager_process_10Hz,
    .module_process_100Hz = uart_manager_process_100Hz,
    .module_process_1kHz  = NULL,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uint8_t cmd_buffer[UART_CMD_MAX_LENGTH];
STATIC uart_cmd_callback_t cmd_callback  = NULL;
STATIC uint32_t rx_commands_received     = 0U;
STATIC volatile bool command_in_progress = false;

// Recursion guard: prevent error_handler from using UART if UART is calling error_handler
STATIC volatile bool in_error_handler_call = false;

typedef enum { IMU_STREAM_OFF = 0, IMU_STREAM_AVG, IMU_STREAM_ARRAY } imu_stream_mode_e;
STATIC volatile imu_stream_mode_e imu_stream_mode = IMU_STREAM_OFF;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void uart_manager_on_rx_line(const char* line, uint16_t length);
STATIC uint16_t uart_manager_trim_command(uint16_t length);
STATIC void uart_manager_default_cmd_handler(const char* cmd, uint16_t length);

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
void uart_manager_imu_stream_enable(void)
{
    imu_stream_mode = IMU_STREAM_ARRAY;
}

void uart_manager_imu_stream_enable_avg(void)
{
    imu_stream_mode = IMU_STREAM_AVG;
}

void uart_manager_imu_stream_disable(void)
{
    imu_stream_mode = IMU_STREAM_OFF;
}

STATIC void uart_manager_process_10Hz(void)
{
    if (imu_stream_mode == IMU_STREAM_OFF)
    {
        return;
    }

    imu_data_t avg;
    bool has_avg = imu_get_data(&avg);

    if (imu_stream_mode == IMU_STREAM_AVG)
    {
        if (has_avg)
        {
            uart_manager_print("AVG A(%+.3f,%+.3f,%+.3f) G(%+.3f,%+.3f,%+.3f) T=%.1fC\r\n",
                               avg.accel.x, avg.accel.y, avg.accel.z,
                               avg.gyro.x, avg.gyro.y, avg.gyro.z, avg.temperature);
        }
        else
        {
            uart_manager_print("IMU not active\r\n");
        }
        return;
    }

    // IMU_STREAM_ARRAY
    uint8_t n      = imu_get_device_count();
    uint8_t active = imu_get_active_count();
    uart_manager_print("IMU[%u/%u]", active, n);

    for (uint8_t i = 0; i < n; i++)
    {
        imu_data_t d;
        if (imu_get_individual_data((imu_device_e)i, &d))
        {
            uart_manager_print(" | [%u] A(%+.2f,%+.2f,%+.2f) G(%+.2f,%+.2f,%+.2f)",
                               i, d.accel.x, d.accel.y, d.accel.z,
                               d.gyro.x, d.gyro.y, d.gyro.z);
        }
        else
        {
            uart_manager_print(" | [%u] inactive", i);
        }
    }

    if (has_avg)
    {
        uart_manager_print(" | AVG A(%+.2f,%+.2f,%+.2f) G(%+.2f,%+.2f,%+.2f)\r\n",
                           avg.accel.x, avg.accel.y, avg.accel.z,
                           avg.gyro.x, avg.gyro.y, avg.gyro.z);
    }
    else
    {
        uart_manager_print("\r\n");
    }
}

STATIC void uart_manager_on_rx_line(const char* line, uint16_t length)
{
    if (line == NULL || length >= UART_CMD_MAX_LENGTH)
    {
        return;
    }
    memcpy(cmd_buffer, line, length);
    cmd_buffer[length] = '\0';
    uint16_t trimmed = uart_manager_trim_command(length);
    if (trimmed == 0)
    {
        return;
    }
    rx_commands_received++;
    char cmd_copy[UART_CMD_MAX_LENGTH];
    memcpy(cmd_copy, cmd_buffer, trimmed + 1);
    command_in_progress = true;
    if (cmd_callback != NULL)
    {
        cmd_callback((const char*)cmd_copy, trimmed);
    }
    command_in_progress = false;
}

STATIC void uart_manager_init(void)
{
    uart_cmd_router_init();
    uart_manager_register_cmd_callback(uart_manager_default_cmd_handler);
    uart_driver_register_rx_line_callback(uart_manager_on_rx_line);
    uart_driver_console_init();
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
        system_driver_reset();
    }
    else
    {
        // Otherwise, route to command router
        uart_cmd_router_dispatch(cmd);
    }
}

STATIC void uart_manager_create_task(void)
{
    /* TX task is created by uart_driver_console_init() */
}

STATIC void uart_manager_process_100Hz(void)
{
    uart_driver_poll_rx();
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

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool uart_manager_transmit(const uint8_t* data, size_t length)
{
    if (data == NULL || length == 0U || length > UART_TX_MSG_MAX_LENGTH)
    {
        return false;
    }
    return uart_driver_send(data, length);
}

bool uart_manager_print(const char* format, ...)
{
    if (format == NULL)
    {
        return false;
    }
    if (in_error_handler_call)
    {
        return false;
    }
    char buffer[UART_PRINT_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, UART_PRINT_BUFFER_SIZE, format, args);
    va_end(args);
    if (len < 0)
    {
        return false;
    }
    if (len >= (int)UART_PRINT_BUFFER_SIZE)
    {
        len = (int)UART_PRINT_BUFFER_SIZE - 1;
    }
    return uart_driver_send((const uint8_t*)buffer, (size_t)len);
}

uint32_t uart_manager_get_queue_count(void)
{
    return uart_driver_get_tx_queue_count();
}

uint32_t uart_manager_get_dropped_count(void)
{
    return uart_driver_get_tx_dropped_count();
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
    return uart_driver_get_rx_overruns();
}

uint32_t uart_manager_get_tx_errors(void)
{
    return uart_driver_get_tx_errors();
}
