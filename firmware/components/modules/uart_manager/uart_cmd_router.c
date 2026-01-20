/*---------------------------------------------------------------------------
 * @file    uart_cmd_router.c
 * @brief   UART command router - parses commands and calls module APIs
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_cmd_router.h"
#include "datalogger.h"
#include "error_handler.h"
#include "imu.h"
#include "ranging/ranging.h"
#include "stopwatch.h"
#include "test_beacon.h"
#include "uart_manager.h"
#include "uwb.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define MAX_CMD_LENGTH 128U

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC const char* skip_whitespace(const char* str);
STATIC void uart_cmd_router_handle_help(void);
STATIC void uart_cmd_router_handle_list(void);
STATIC void uart_cmd_router_handle_beacon(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_imu(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_uwb(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_error(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_datalogger(const char* action, const char* target,
                                              const char* args);
STATIC void uart_cmd_router_handle_ranging(const char* action, const char* target,
                                           const char* args);
STATIC void uart_cmd_router_handle_stopwatch(const char* action, const char* target,
                                             const char* args);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC bool router_initialized = false;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC const char* skip_whitespace(const char* str)
{
    while (*str && isspace((unsigned char)*str))
    {
        str++;
    }
    return str;
}

STATIC void uart_cmd_router_handle_help(void)
{
    uart_manager_print("\r\nAvailable Commands:\r\n");
    uart_manager_print("  help  - Show this help message\r\n");
    uart_manager_print("  list  - List all available modules\r\n");
    uart_manager_print("\r\nCommand format: <module>.<action>.<target> [args]\r\n");
    uart_manager_print("Example: beacon.get.status\r\n");
    uart_manager_print("Example: beacon.set.mode responder\r\n");
    uart_manager_print("Example: beacon.req.ping 0x0002\r\n\r\n");
}

STATIC void uart_cmd_router_handle_list(void)
{
    uart_manager_print("\r\nAvailable Modules:\r\n");
    uart_manager_print("  beacon     - Test beacon module\r\n");
    uart_manager_print("  imu        - IMU sensor module\r\n");
    uart_manager_print("  uwb        - UWB radio module\r\n");
    uart_manager_print("  ranging    - TWR ranging module\r\n");
    uart_manager_print("  error      - Error handler module\r\n");
    uart_manager_print("  datalogger - System monitoring\r\n");
    uart_manager_print("  stopwatch  - Performance timing (0-9 instances)\r\n\r\n");
}

STATIC void uart_cmd_router_handle_beacon(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            beacon_status_t status;
            test_beacon_get_status(&status);
            const char* mode_str = (status.mode == BEACON_MODE_RESPONDER) ? "responder" : "master";
            uart_manager_print("Beacon mode: %s\r\n", mode_str);
        }
        else if (strcmp(target, "stats") == 0)
        {
            beacon_status_t status;
            test_beacon_get_status(&status);
            const char* mode_str = (status.mode == BEACON_MODE_RESPONDER) ? "responder" : "master";
            uart_manager_print("Beacon stats:\r\n");
            uart_manager_print("  Mode: %s\r\n", mode_str);
            uart_manager_print("  Counter: %u\r\n", status.counter);
            uart_manager_print("  RX: %u, TX: %u\r\n", (unsigned int)status.rx_count,
                               (unsigned int)status.tx_count);
            uart_manager_print("  Last sender: 0x%04X\r\n", status.last_src_addr);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "mode") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: beacon.set.mode <responder|master>\r\n");
                return;
            }
            beacon_mode_e mode;
            if (strcmp(args, "responder") == 0)
            {
                mode = BEACON_MODE_RESPONDER;
            }
            else if (strcmp(args, "master") == 0)
            {
                mode = BEACON_MODE_MASTER;
            }
            else
            {
                uart_manager_print("ERR: Invalid mode. Use 'responder' or 'master'\r\n");
                return;
            }
            test_beacon_set_mode(mode);
            uart_manager_print("Mode set to %s\r\n", args);
        }
        else if (strcmp(target, "counter") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: beacon.set.counter <value>\r\n");
                return;
            }
            int value = atoi(args);
            if (value < 0 || value > 65535)
            {
                uart_manager_print("ERR: Value out of range (0-65535)\r\n");
                return;
            }
            test_beacon_set_counter((uint16_t)value);
            uart_manager_print("Counter set to %u\r\n", value);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "req") == 0)
    {
        if (strcmp(target, "ping") == 0)
        {
            uint16_t dest_addr = 0xFFFF;
            if (args != NULL && *args != '\0')
            {
                dest_addr = (uint16_t)strtol(args, NULL, 0);
            }
            uart_manager_print("Sending ping to 0x%04X...\r\n", dest_addr);
            if (test_beacon_send_ping(dest_addr))
            {
                uart_manager_print("Ping sent\r\n");
            }
            else
            {
                uart_manager_print("Ping failed\r\n");
            }
        }
        else if (strcmp(target, "ping.delayed") == 0)
        {
            uint16_t dest_addr = 0xFFFF;
            if (args != NULL && *args != '\0')
            {
                dest_addr = (uint16_t)strtol(args, NULL, 0);
            }
            uart_manager_print("Sending delayed ping to 0x%04X...\r\n", dest_addr);
            if (test_beacon_send_ping_delayed(dest_addr))
            {
                uart_manager_print("Delayed ping sent\r\n");
            }
            else
            {
                uart_manager_print("Delayed ping failed\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_imu(const char* action, const char* target, const char* args)
{
    (void)args;

    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            imu_status_t status;
            imu_get_status(&status);
            const char* state_str;
            switch (status.state)
            {
                case IMU_STATE_ACTIVE:
                    state_str = "active";
                    break;
                case IMU_STATE_INITIALIZATION:
                    state_str = "init";
                    break;
                case IMU_STATE_FAULTED:
                    state_str = "FAULTED";
                    break;
                default:
                    state_str = "startup";
                    break;
            }
            uart_manager_print("IMU: %s, chip_id=0x%02X\r\n", state_str, status.chip_id);
        }
        else if (strcmp(target, "data") == 0)
        {
            imu_data_t data;
            if (imu_get_data(&data))
            {
                uart_manager_print("Accel: X=%.3f Y=%.3f Z=%.3f m/s^2\r\n", data.accel.x,
                                   data.accel.y, data.accel.z);
                uart_manager_print("Gyro:  X=%.3f Y=%.3f Z=%.3f deg/s\r\n", data.gyro.x,
                                   data.gyro.y, data.gyro.z);
                uart_manager_print("Temp:  %.2f C\r\n", data.temperature);
            }
            else
            {
                uart_manager_print("IMU not active\r\n");
            }
        }
        else if (strcmp(target, "accel") == 0)
        {
            imu_vector3_t accel;
            if (imu_get_accel(&accel))
            {
                uart_manager_print("Accel: X=%.3f Y=%.3f Z=%.3f m/s^2\r\n", accel.x, accel.y,
                                   accel.z);
            }
            else
            {
                uart_manager_print("IMU not active\r\n");
            }
        }
        else if (strcmp(target, "gyro") == 0)
        {
            imu_vector3_t gyro;
            if (imu_get_gyro(&gyro))
            {
                uart_manager_print("Gyro: X=%.3f Y=%.3f Z=%.3f deg/s\r\n", gyro.x, gyro.y, gyro.z);
            }
            else
            {
                uart_manager_print("IMU not active\r\n");
            }
        }
        else if (strcmp(target, "temp") == 0)
        {
            float temp;
            if (imu_get_temp(&temp))
            {
                uart_manager_print("Temp: %.2f C\r\n", temp);
            }
            else
            {
                uart_manager_print("IMU not active\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_uwb(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "address") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: uwb.set.address <addr> [pan_id]\r\n");
                return;
            }

            // Parse address (required)
            char* end_ptr;
            uint16_t address = (uint16_t)strtoul(args, &end_ptr, 0);

            // Parse optional PAN ID
            uint16_t pan_id = 0xDECA; // Default
            if (end_ptr && *end_ptr != '\0')
            {
                const char* pan_str = skip_whitespace(end_ptr);
                if (*pan_str != '\0')
                {
                    pan_id = (uint16_t)strtoul(pan_str, NULL, 0);
                }
            }

            uwb_set_address(address, pan_id);
            uart_manager_print("UWB address set to: ADDR=0x%04X, PAN=0x%04X\r\n", address, pan_id);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            uwb_status_t status;
            uwb_get_status(&status);
            const char* state_str;
            switch (status.state)
            {
                case UWB_STATE_ACTIVE:
                    state_str = "active";
                    break;
                case UWB_STATE_INITIALIZATION:
                    state_str = "init";
                    break;
                case UWB_STATE_OFF:
                    state_str = "off";
                    break;
                case UWB_STATE_FAULTED:
                    state_str = "FAULTED";
                    break;
                default:
                    state_str = "unknown";
                    break;
            }
            uart_manager_print("UWB: %s, dev_id=0x%08X\r\n", state_str,
                               (unsigned int)status.device_id);
        }
        else if (strcmp(target, "addr") == 0)
        {
            uwb_status_t status;
            uwb_get_status(&status);
            uart_manager_print("Addr: PAN=0x%04X, ADDR=0x%04X\r\n", status.my_pan_id,
                               status.my_address);
        }
        else if (strcmp(target, "stats") == 0)
        {
            uwb_rx_stats_t stats;
            uwb_get_rx_stats(&stats);
            uart_manager_print("RX: %u received, %u errors, %u filtered\r\n",
                               (unsigned int)stats.received, (unsigned int)stats.rx_errors,
                               (unsigned int)stats.filtered);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "req") == 0)
    {
        if (strcmp(target, "start") == 0)
        {
            uwb_start();
            uart_manager_print("UWB start requested\r\n");
        }
        else if (strcmp(target, "stop") == 0)
        {
            uwb_stop();
            uart_manager_print("UWB stop requested\r\n");
        }
        else if (strcmp(target, "resetstats") == 0)
        {
            uwb_reset_rx_stats();
            uart_manager_print("Stats reset\r\n");
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_error(const char* action, const char* target, const char* args)
{
    (void)args;

    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            uart_manager_print("\r\nError Status:\r\n");
            uart_manager_print("INFO:    %lu\r\n",
                               (unsigned long)error_handler_get_count(ERROR_SEVERITY_INFO));
            uart_manager_print("WARNING: %lu\r\n",
                               (unsigned long)error_handler_get_count(ERROR_SEVERITY_WARNING));
            uart_manager_print("ERROR:   %lu\r\n",
                               (unsigned long)error_handler_get_count(ERROR_SEVERITY_ERROR));
            uart_manager_print("FATAL:   %lu\r\n",
                               (unsigned long)error_handler_get_count(ERROR_SEVERITY_FATAL));
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "clear") == 0)
        {
            error_handler_clear_history();
            uart_manager_print("Error history cleared\r\n");
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_datalogger(const char* action, const char* target,
                                              const char* args)
{
    (void)args;

    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "tasks") == 0)
        {
            uart_manager_print("\r\nTask List:\r\n");
            task_cpu_info_t tasks[20];
            uint32_t count = datalogger_get_task_usage(tasks, 20);
            for (uint32_t i = 0; i < count; i++)
            {
                uart_manager_print("%-20s %5.2f%%\r\n", tasks[i].task_name, tasks[i].cpu_percent);
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_stopwatch(const char* action, const char* target,
                                             const char* args)
{
    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "all") == 0)
        {
            uart_manager_print("\r\nStopwatch Values:\r\n");
            for (uint8_t i = 0; i < 10; i++)
            {
                uint32_t elapsed = stopwatch_elapsed_us(i);
                bool running = stopwatch_is_running(i);
                uart_manager_print("  [%u] %lu us %s\r\n", i, (unsigned long)elapsed,
                                   running ? "(running)" : "(stopped)");
            }
        }
        else
        {
            // Parse stopwatch ID
            uint8_t id = (uint8_t)strtoul(target, NULL, 0);
            if (id >= 10)
            {
                uart_manager_print("ERR: Invalid stopwatch ID (0-9)\r\n");
                return;
            }
            uint32_t elapsed = stopwatch_elapsed_us(id);
            bool running = stopwatch_is_running(id);
            uart_manager_print("Stopwatch %u: %lu us %s\r\n", id, (unsigned long)elapsed,
                               running ? "(running)" : "(stopped)");
        }
    }
    else if (strcmp(action, "start") == 0)
    {
        uint8_t id = (uint8_t)strtoul(target, NULL, 0);
        if (id >= 10)
        {
            uart_manager_print("ERR: Invalid stopwatch ID (0-9)\r\n");
            return;
        }
        stopwatch_start(id);
        uart_manager_print("Stopwatch %u started\r\n", id);
    }
    else if (strcmp(action, "stop") == 0)
    {
        uint8_t id = (uint8_t)strtoul(target, NULL, 0);
        if (id >= 10)
        {
            uart_manager_print("ERR: Invalid stopwatch ID (0-9)\r\n");
            return;
        }
        stopwatch_stop(id);
        uint32_t elapsed = stopwatch_elapsed_us(id);
        uart_manager_print("Stopwatch %u stopped: %lu us\r\n", id, (unsigned long)elapsed);
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

STATIC void uart_cmd_router_handle_ranging(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "mode") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Mode argument required (tag/anchor/disabled)\r\n");
                return;
            }

            ranging_mode_e mode;
            if (strcmp(args, "tag") == 0)
            {
                mode = RANGING_MODE_TAG;
            }
            else if (strcmp(args, "anchor") == 0)
            {
                mode = RANGING_MODE_ANCHOR;
            }
            else if (strcmp(args, "disabled") == 0)
            {
                mode = RANGING_MODE_DISABLED;
            }
            else
            {
                uart_manager_print("ERR: Invalid mode. Use: tag, anchor, or disabled\r\n");
                return;
            }

            if (ranging_set_mode(mode))
            {
                const char* mode_str = (mode == RANGING_MODE_TAG)      ? "TAG"
                                       : (mode == RANGING_MODE_ANCHOR) ? "ANCHOR"
                                                                       : "DISABLED";
                uart_manager_print("Ranging mode set to: %s\r\n", mode_str);
            }
            else
            {
                uart_manager_print("ERR: Failed to set ranging mode\r\n");
            }
        }
        else if (strcmp(target, "address") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Address argument required (hex: 0xABCD)\r\n");
                return;
            }

            uint16_t address = (uint16_t)strtoul(args, NULL, 0);

            // Set at UWB level for both modes
            uwb_set_address(address, 0xDECA);

            ranging_mode_e current_mode = ranging_get_mode();
            if (current_mode == RANGING_MODE_TAG)
            {
                uart_manager_print("Tag address set to: 0x%04X\r\n", address);
            }
            else if (current_mode == RANGING_MODE_ANCHOR)
            {
                uart_manager_print("Anchor address set to: 0x%04X\r\n", address);
            }
            else
            {
                uart_manager_print("Address set to: 0x%04X\r\n", address);
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            ranging_status_t status;
            ranging_get_status(&status);

            const char* mode_str = (status.mode == RANGING_MODE_TAG)      ? "TAG"
                                   : (status.mode == RANGING_MODE_ANCHOR) ? "ANCHOR"
                                                                          : "DISABLED";

            uart_manager_print("Ranging Status:\r\n");
            uart_manager_print("  Mode: %s\r\n", mode_str);
            uart_manager_print("  Active: %s\r\n", status.active ? "Yes" : "No");
            uart_manager_print("  Successful: %lu\r\n", (unsigned long)status.successful_ranges);
            uart_manager_print("  Failed: %lu\r\n", (unsigned long)status.failed_ranges);

            if (status.mode == RANGING_MODE_ANCHOR)
            {
                uint16_t addr = ranging_anchor_get_address();
                uart_manager_print("  Anchor Address: 0x%04X\r\n", addr);
            }
            else if (status.mode == RANGING_MODE_TAG)
            {
                uint16_t addr = uwb_get_address();
                uart_manager_print("  Tag Address: 0x%04X\r\n", addr);
            }
        }
        else if (strcmp(target, "result") == 0)
        {
            float distance, rssi;
            if (ranging_tag_get_result(&distance, &rssi))
            {
                uart_manager_print("Last Range: %.2f m, RSSI: %.1f dBm\r\n", distance, rssi);
            }
            else
            {
                uart_manager_print("No ranging result available\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "req") == 0)
    {
        if (strcmp(target, "range") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Anchor address required (hex: 0xABCD)\r\n");
                return;
            }

            uint16_t anchor_addr = (uint16_t)strtoul(args, NULL, 0);

            if (!ranging_tag_start(anchor_addr))
            {
                uart_manager_print("ERR: Failed to start ranging (check mode and UWB status)\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action '%s'\r\n", action);
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void uart_cmd_router_init(void)
{
    if (router_initialized)
    {
        return;
    }
    router_initialized = true;
}

void uart_cmd_router_dispatch(const char* cmd_string)
{
    if (cmd_string == NULL)
    {
        uart_manager_print("ERR: Invalid command\r\n");
        return;
    }

    char local_buffer[MAX_CMD_LENGTH];
    strncpy(local_buffer, cmd_string, sizeof(local_buffer) - 1);
    local_buffer[sizeof(local_buffer) - 1] = '\0';

    char* p = local_buffer;
    while (*p && isspace((unsigned char)*p))
    {
        p++;
    }

    if (*p == '\0')
    {
        return;
    }

    if (strcmp(p, "help") == 0)
    {
        uart_cmd_router_handle_help();
        return;
    }
    if (strcmp(p, "list") == 0)
    {
        uart_cmd_router_handle_list();
        return;
    }

    char* module = p;
    char* dot1 = strchr(module, '.');
    if (dot1 == NULL)
    {
        uart_manager_print("ERR: Invalid format. Use module.action.target\r\n");
        return;
    }
    *dot1 = '\0';
    char* action = dot1 + 1;

    char* dot2 = strchr(action, '.');
    if (dot2 == NULL)
    {
        uart_manager_print("ERR: Invalid format. Use module.action.target\r\n");
        return;
    }
    *dot2 = '\0';
    char* target = dot2 + 1;

    char* args = NULL;
    char* space = strchr(target, ' ');
    if (space != NULL)
    {
        *space = '\0';
        args = (char*)skip_whitespace(space + 1);
    }

    if (strcmp(module, "beacon") == 0)
    {
        uart_cmd_router_handle_beacon(action, target, args);
    }
    else if (strcmp(module, "imu") == 0)
    {
        uart_cmd_router_handle_imu(action, target, args);
    }
    else if (strcmp(module, "uwb") == 0)
    {
        uart_cmd_router_handle_uwb(action, target, args);
    }
    else if (strcmp(module, "error") == 0)
    {
        uart_cmd_router_handle_error(action, target, args);
    }
    else if (strcmp(module, "datalogger") == 0)
    {
        uart_cmd_router_handle_datalogger(action, target, args);
    }
    else if (strcmp(module, "ranging") == 0)
    {
        uart_cmd_router_handle_ranging(action, target, args);
    }
    else if (strcmp(module, "stopwatch") == 0)
    {
        uart_cmd_router_handle_stopwatch(action, target, args);
    }
    else
    {
        uart_manager_print("ERR: Unknown module '%s'\r\n", module);
    }
}
