/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uart_cmd_router.h"
#include "FreeRTOS.h"
#include "common.h"
#include "datalogger.h"
#include "error_handler.h"
#include "imu.h"
#include "ota_config.h"
#include "platform_system.h"
#include "sensor_fusion.h"
#include "stopwatch.h"
#include "task.h"
#include "twr_types.h"
#include "initiator.h"
#include "responder.h"
#include "twr_manager.h"
#include "twr_scheduler.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"

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
STATIC void uart_cmd_router_handle_node(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_uwb(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_error(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_datalogger(const char* action, const char* target,
                                              const char* args);
STATIC void uart_cmd_router_handle_twr(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_twr_manager(const char* action, const char* target,
                                               const char* args);
STATIC void uart_cmd_router_handle_stopwatch(const char* action, const char* target,
                                             const char* args);
STATIC void uart_cmd_router_handle_ota_config(const char* action, const char* target,
                                              const char* args);
STATIC void uart_cmd_router_handle_system(const char* action, const char* target, const char* args);
STATIC void uart_cmd_router_handle_sensor_fusion(const char* action, const char* target,
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
    uart_manager_print("  system     - System information (UUID, device ID)\r\n");
    uart_manager_print("  data       - Data communications module\r\n");
    uart_manager_print("  imu        - IMU sensor module\r\n");
    uart_manager_print("  uwb_node   - UWB node identity and configuration\r\n");
    uart_manager_print("  uwb        - UWB radio transceiver\r\n");
    uart_manager_print("  twr        - Two-Way Ranging service\r\n");
    uart_manager_print("  twrmgr - Two-Way Ranging manager\r\n");
    uart_manager_print("  ota_config - OTA configuration (remote node programming)\r\n");
    uart_manager_print("  sf         - Sensor Fusion (position estimation)\r\n");
    uart_manager_print("  error      - Error handler module\r\n");
    uart_manager_print("  datalogger - System monitoring\r\n");
    uart_manager_print("  stopwatch  - Performance timing (0-9 instances)\r\n\r\n");
}

STATIC void uart_cmd_router_handle_beacon(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "req") == 0)
    {
        if (strcmp(target, "ping") == 0)
        {
            uint16_t dest_addr = 0xFFFF;
            if (args != NULL && *args != '\0')
            {
                dest_addr = (uint16_t)strtol(args, NULL, 0);
            }

            // Construct DATA protocol beacon message
            protocol_header_t msg = {
                .protocol_type = PROTOCOL_TYPE_DATA, .msg_type = DATA_MSG_BEACON, .sequence = 0};

            uart_manager_print("Sending DATA beacon to 0x%04X...\r\n", dest_addr);
            uwb_send_result_t result =
                uwb_send_message((const uint8_t*)&msg, sizeof(msg), dest_addr);
            if (result.success)
            {
                uart_manager_print("Beacon sent\r\n");
            }
            else
            {
                uart_manager_print("Beacon failed\r\n");
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
            vec3_t accel;
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
            vec3_t gyro;
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

STATIC void uart_cmd_router_handle_node(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "type") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("ERR: Node type required (tag, anchor, or hybrid)\r\n");
                return;
            }

            uwb_node_type_e type;
            if (strcmp(args, "tag") == 0)
            {
                type = UWB_NODE_TYPE_TAG;
            }
            else if (strcmp(args, "anchor") == 0)
            {
                type = UWB_NODE_TYPE_ANCHOR;
            }
            else if (strcmp(args, "hybrid") == 0)
            {
                type = UWB_NODE_TYPE_HYBRID;
            }
            else
            {
                uart_manager_print("ERR: Invalid node type. Use: tag, anchor, or hybrid\r\n");
                return;
            }

            uwb_node_set_type(type);
            const char* type_str = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                                   : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                                                    : "HYBRID";
            uart_manager_print("Node type set to: %s\r\n", type_str);
        }
        else if (strcmp(target, "position") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("ERR: Position required (x y z in meters)\r\n");
                return;
            }

            // Parse three floats: x, y, z using strtof
            char* end_ptr;
            float x = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid X coordinate\r\n");
                return;
            }

            args    = skip_whitespace(end_ptr);
            float y = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid Y coordinate\r\n");
                return;
            }

            args    = skip_whitespace(end_ptr);
            float z = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid Z coordinate\r\n");
                return;
            }

            vec3_t pos = {.x = x, .y = y, .z = z};
            uwb_node_set_position(&pos);
            uart_manager_print("Node position set to: X=%.2f Y=%.2f Z=%.2f m\r\n", x, y, z);
        }
        else if (strcmp(target, "address") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("ERR: Address required (hex: 0xABCD)\r\n");
                return;
            }

            uint16_t address = (uint16_t)strtoul(args, NULL, 0);
            uwb_set_address(address, uwb_get_pan_id());
            uart_manager_print("Node address set to: 0x%04X\r\n", address);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "type") == 0)
        {
            uwb_node_type_e type = uwb_node_get_type();
            const char* type_str = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                                   : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                   : (type == UWB_NODE_TYPE_HYBRID) ? "HYBRID"
                                                                    : "UNKNOWN";
            uart_manager_print("Node type: %s\r\n", type_str);
        }
        else if (strcmp(target, "position") == 0)
        {
            vec3_t pos;
            if (uwb_node_get_position(&pos))
            {
                uart_manager_print("Node position: X=%.2f Y=%.2f Z=%.2f m\r\n", pos.x, pos.y,
                                   pos.z);
            }
            else
            {
                uart_manager_print("Position not set\r\n");
            }
        }
        else if (strcmp(target, "address") == 0)
        {
            uint16_t address = uwb_get_address();
            uart_manager_print("Node address: 0x%04X\r\n", address);
        }
        else if (strcmp(target, "status") == 0)
        {
            uwb_node_type_e type = uwb_node_get_type();
            const char* type_str = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                                   : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                   : (type == UWB_NODE_TYPE_HYBRID) ? "HYBRID"
                                                                    : "UNKNOWN";

            vec3_t pos;
            bool has_position = uwb_node_get_position(&pos);

            uart_manager_print("Node Status:\r\n");
            uart_manager_print("  Type: %s\r\n", type_str);
            if (has_position)
            {
                uart_manager_print("  Position: X=%.2f Y=%.2f Z=%.2f m\r\n", pos.x, pos.y, pos.z);
            }
            else
            {
                uart_manager_print("  Position: Not set\r\n");
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
            uart_manager_print("RX: %u received, %u filtered\r\n", (unsigned int)stats.received,
                               (unsigned int)stats.filtered);
            uart_manager_print("Queue: TX overflows=%u, RX overflows=%u\r\n",
                               (unsigned int)uwb_get_tx_queue_overflows(),
                               (unsigned int)uwb_get_rx_queue_overflows());

            uwb_port_statistics_t port_stats = uwb_port_get_statistics();
            uart_manager_print("Port Stats:\r\n");
            uart_manager_print("  rx_ok_count: %lu\r\n", (unsigned long)port_stats.rx_ok_count);
            uart_manager_print("  rx_timeout_count: %lu\r\n",
                               (unsigned long)port_stats.rx_timeout_count);
            uart_manager_print("  tx_done_count: %lu\r\n", (unsigned long)port_stats.tx_done_count);
            uart_manager_print("  send_message_count: %lu\r\n",
                               (unsigned long)port_stats.send_message_count);
            uart_manager_print("HW Diagnostic Counters:\r\n");
            uart_manager_print("  PHE (PHY header errors): %u\r\n", port_stats.PHE);
            uart_manager_print("  RSL (sync loss): %u\r\n", port_stats.RSL);
            uart_manager_print("  CRCG (good CRC): %u\r\n", port_stats.CRCG);
            uart_manager_print("  CRCB (bad CRC): %u\r\n", port_stats.CRCB);
            uart_manager_print("  ARFE (addr filter err): %u\r\n", port_stats.ARFE);
            uart_manager_print("  OVER (RX overrun): %u\r\n", port_stats.OVER);
            uart_manager_print("  SFDTO (SFD timeout): %u\r\n", port_stats.SFDTO);
            uart_manager_print("  PTO (preamble timeout): %u\r\n", port_stats.PTO);
            uart_manager_print("  RTO (RX timeout): %u\r\n", port_stats.RTO);
            uart_manager_print("  TXF (TX frames): %u\r\n", port_stats.TXF);
            uart_manager_print("Status Registers:\r\n");
            uart_manager_print("  irq_status: 0x%08lX\r\n", (unsigned long)port_stats.irq_status);
            uart_manager_print("  status_lo: 0x%08lX\r\n", (unsigned long)port_stats.status_lo);
            uart_manager_print("  status_hi: 0x%08lX\r\n", (unsigned long)port_stats.status_hi);
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
        else if (strcmp(target, "stats") == 0)
        {
            uart_manager_print("\r\nSystem Statistics:\r\n");

            system_stats_t stats;
            task_cpu_info_t task_buffer[20];
            datalogger_get_system_stats(&stats, task_buffer, 20);

            uart_manager_print("Memory:\r\n");
            uart_manager_print("  Current Free:     %u bytes\r\n",
                               (unsigned int)stats.current_free_heap);
            uart_manager_print("  Minimum Ever:     %u bytes\r\n",
                               (unsigned int)stats.minimum_ever_free_heap);
            uart_manager_print("  Total Heap:       %u bytes\r\n",
                               (unsigned int)stats.total_heap_size);
            uart_manager_print(
                "  Used:             %u bytes (%.1f%%)\r\n",
                (unsigned int)(stats.total_heap_size - stats.current_free_heap),
                100.0f * (stats.total_heap_size - stats.current_free_heap) / stats.total_heap_size);

            uart_manager_print("\r\nTask CPU Usage:\r\n");
            for (uint32_t i = 0; i < stats.num_tasks; i++)
            {
                uart_manager_print("  %-18s %5.2f%%\r\n", stats.task_info[i].task_name,
                                   stats.task_info[i].cpu_percent);
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

STATIC void uart_cmd_router_handle_twr_manager(const char* action, const char* target,
                                               const char* args)
{
    (void)args; // Most commands don't need args

    if (strcmp(action, "req") == 0)
    {
        if (strcmp(target, "start") == 0)
        {
            if (twr_manager_start())
            {
                uart_manager_print("TWR manager started (auto-ranging enabled)\r\n");
            }
            else
            {
                uart_manager_print("ERR: Failed to start TWR manager\r\n");
            }
        }
        else if (strcmp(target, "stop") == 0)
        {
            twr_manager_stop();
            uart_manager_print("TWR manager stopped\r\n");
        }
        else
        {
            uart_manager_print("ERR: Unknown request. Use: start, stop\r\n");
        }
    }
    else if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "status") == 0)
        {
            bool active      = twr_manager_is_active();
            uint32_t success = twr_manager_get_success_count();
            uint32_t failure = twr_manager_get_failure_count();
            float success_rate =
                (success + failure > 0) ? (100.0f * success / (success + failure)) : 0.0f;

            // Get scheduler status
            twr_scheduler_status_t sched_status;
            twr_scheduler_get_status(&sched_status);

            uart_manager_print("TWR Manager Status:\r\n");
            uart_manager_print("  Active: %s\r\n", active ? "YES" : "NO");
            uart_manager_print("  Success: %u\r\n", (unsigned int)success);
            uart_manager_print("  Failure: %u\r\n", (unsigned int)failure);
            uart_manager_print("  Success Rate: %.1f%%\r\n", success_rate);
            uart_manager_print("  Targets Configured: %d\r\n", sched_status.target_count);
            uart_manager_print("  Current Target: 0x%04X\r\n", sched_status.current_target);
        }
        else
        {
            uart_manager_print("ERR: Unknown get target. Use: status\r\n");
        }
    }
    else if (strcmp(action, "add") == 0)
    {
        // Accept both 'anchor' (backward compat) and 'target' (new)
        if (strcmp(target, "anchor") == 0 || strcmp(target, "target") == 0)
        {
            if (args == NULL || args[0] == '\0')
            {
                uart_manager_print("ERR: Missing target address (e.g., 0x0001)\r\n");
                return;
            }

            // Parse address
            uint16_t address = (uint16_t)strtoul(args, NULL, 0);

            if (twr_scheduler_add_target(address))
            {
                uart_manager_print("Added target 0x%04X (total: %d)\r\n", address,
                                   twr_scheduler_get_target_count());
            }
            else
            {
                uart_manager_print("ERR: Failed to add target 0x%04X\r\n", address);
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown add target. Use: target or anchor\r\n");
        }
    }
    else if (strcmp(action, "remove") == 0)
    {
        // Accept both 'anchor' (backward compat) and 'target' (new)
        if (strcmp(target, "anchor") == 0 || strcmp(target, "target") == 0)
        {
            if (args == NULL || args[0] == '\0')
            {
                uart_manager_print("ERR: Missing target address (e.g., 0x0001)\r\n");
                return;
            }

            uint16_t address = (uint16_t)strtoul(args, NULL, 0);

            if (twr_scheduler_remove_target(address))
            {
                uart_manager_print("Removed target 0x%04X (remaining: %d)\r\n", address,
                                   twr_scheduler_get_target_count());
            }
            else
            {
                uart_manager_print("ERR: Target 0x%04X not found\r\n", address);
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown remove target. Use: target or anchor\r\n");
        }
    }
    else if (strcmp(action, "set") == 0)
    {
        // Accept both 'anchors' (backward compat) and 'targets' (new)
        if (strcmp(target, "anchors") == 0 || strcmp(target, "targets") == 0)
        {
            if (args == NULL || args[0] == '\0')
            {
                uart_manager_print(
                    "ERR: Missing target addresses (e.g., 0x0001 0x0002 0x0003)\r\n");
                return;
            }

            // Parse multiple addresses
            uint16_t addresses[TWR_SCHEDULER_MAX_TARGETS];
            uint8_t count   = 0;
            const char* ptr = args;

            while (*ptr != '\0' && count < TWR_SCHEDULER_MAX_TARGETS)
            {
                ptr = skip_whitespace(ptr);
                if (*ptr == '\0')
                    break;

                addresses[count] = (uint16_t)strtoul(ptr, (char**)&ptr, 0);
                count++;
            }

            if (count == 0)
            {
                uart_manager_print("ERR: No valid addresses parsed\r\n");
                return;
            }

            if (twr_scheduler_set_targets(addresses, count))
            {
                uart_manager_print("Set %d targets: ", count);
                for (uint8_t i = 0; i < count; i++)
                {
                    uart_manager_print("0x%04X ", addresses[i]);
                }
                uart_manager_print("\r\n");
            }
            else
            {
                uart_manager_print("ERR: Failed to set targets\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown set target. Use: targets or anchors\r\n");
        }
    }
    else if (strcmp(action, "clear") == 0)
    {
        // Accept both 'anchors' (backward compat) and 'targets' (new)
        if (strcmp(target, "anchors") == 0 || strcmp(target, "targets") == 0)
        {
            twr_scheduler_clear_all();
            uart_manager_print("Cleared all targets\r\n");
        }
        else
        {
            uart_manager_print("ERR: Unknown clear target. Use: targets or anchors\r\n");
        }
    }
    else if (strcmp(action, "setrate") == 0)
    {
        if (strcmp(target, "ranging") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Missing rate argument (Hz)\r\n");
                uart_manager_print("Example: twrmgr.setrate.ranging 100\r\n");
                return;
            }
            
            uint16_t rate_hz = (uint16_t)atoi(args);
            if (twr_manager_set_ranging_rate(rate_hz))
            {
                uart_manager_print("Ranging rate set to %u Hz\r\n", rate_hz);
            }
            else
            {
                uart_manager_print("ERR: Failed to set ranging rate (valid range: 1-200 Hz)\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown setrate target. Use: ranging\r\n");
        }
    }
    else if (strcmp(action, "getrate") == 0)
    {
        if (strcmp(target, "ranging") == 0)
        {
            uint16_t rate_hz = twr_manager_get_ranging_rate();
            uart_manager_print("Current ranging rate: %u Hz\r\n", rate_hz);
        }
        else
        {
            uart_manager_print("ERR: Unknown getrate target. Use: ranging\r\n");
        }
    }
    else
    {
        uart_manager_print("ERR: Unknown action. Use: req, get, add, remove, set, clear, setrate, getrate\r\n");
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
                bool running     = stopwatch_is_running(i);
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
            bool running     = stopwatch_is_running(id);
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

STATIC void uart_cmd_router_handle_ota_config(const char* action, const char* target,
                                              const char* args)
{
    if (strcmp(action, "send") == 0)
    {
        if (strcmp(target, "address") == 0)
        {
            // ota_config.send.address <target_addr> <new_addr> [pan_id]
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: ota_config.send.address <target_addr> <new_addr> "
                                   "[pan_id]\r\n");
                return;
            }

            char* end_ptr;
            uint16_t target_addr = (uint16_t)strtoul(args, &end_ptr, 0);

            if (end_ptr == args || *end_ptr == '\0')
            {
                uart_manager_print("ERR: Missing new_addr parameter\r\n");
                return;
            }

            const char* new_addr_str = skip_whitespace(end_ptr);
            uint16_t new_addr        = (uint16_t)strtoul(new_addr_str, &end_ptr, 0);

            uint16_t pan_id = 0xFFFF; // Keep current by default
            if (end_ptr && *end_ptr != '\0')
            {
                const char* pan_str = skip_whitespace(end_ptr);
                if (*pan_str != '\0')
                {
                    pan_id = (uint16_t)strtoul(pan_str, NULL, 0);
                }
            }

            vTaskDelay(pdMS_TO_TICKS(4));

            bool success = ota_config_send_set_address(target_addr, new_addr, pan_id);
            if (success)
            {
                if (pan_id == 0xFFFF)
                {
                    uart_manager_print(
                        "Sent SET_ADDRESS to 0x%04X: new_addr=0x%04X (PAN unchanged)\r\n",
                        target_addr, new_addr);
                }
                else
                {
                    uart_manager_print(
                        "Sent SET_ADDRESS to 0x%04X: new_addr=0x%04X, pan=0x%04X\r\n", target_addr,
                        new_addr, pan_id);
                }
            }
            else
            {
                uart_manager_print("Failed to send SET_ADDRESS\r\n");
            }
        }
        else if (strcmp(target, "position") == 0)
        {
            // ota_config.send.position <target_addr> <x> <y> <z>
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: ota_config.send.position <target_addr> <x> <y> <z>\r\n");
                return;
            }

            char* end_ptr;
            uint16_t target_addr = (uint16_t)strtoul(args, &end_ptr, 0);

            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid target address\r\n");
                return;
            }

            vec3_t position;
            position.x = strtof(skip_whitespace(end_ptr), &end_ptr);
            position.y = strtof(skip_whitespace(end_ptr), &end_ptr);
            position.z = strtof(skip_whitespace(end_ptr), &end_ptr);

            bool success = ota_config_send_set_position(target_addr, &position);
            if (success)
            {
                uart_manager_print("Sent SET_POSITION to 0x%04X: (%.2f, %.2f, %.2f)\r\n",
                                   target_addr, position.x, position.y, position.z);
            }
            else
            {
                uart_manager_print("Failed to send SET_POSITION\r\n");
            }
        }
        else if (strcmp(target, "type") == 0)
        {
            // ota_config.send.type <target_addr> <type>
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: ota_config.send.type <target_addr> <type>\r\n");
                uart_manager_print("  type: 0=TAG, 1=ANCHOR, 2=HYBRID\r\n");
                return;
            }

            char* end_ptr;
            uint16_t target_addr = (uint16_t)strtoul(args, &end_ptr, 0);

            if (end_ptr == args || *end_ptr == '\0')
            {
                uart_manager_print("ERR: Missing type parameter\r\n");
                return;
            }

            const char* type_str = skip_whitespace(end_ptr);
            uint8_t type         = (uint8_t)strtoul(type_str, NULL, 0);

            if (type > UWB_NODE_TYPE_HYBRID)
            {
                uart_manager_print("ERR: Invalid type (0=TAG, 1=ANCHOR, 2=HYBRID)\r\n");
                return;
            }

            bool success = ota_config_send_set_node_type(target_addr, (uwb_node_type_e)type);
            if (success)
            {
                const char* type_name = (type == UWB_NODE_TYPE_TAG)      ? "TAG"
                                        : (type == UWB_NODE_TYPE_ANCHOR) ? "ANCHOR"
                                                                         : "HYBRID";
                uart_manager_print("Sent SET_NODE_TYPE to 0x%04X: %s\r\n", target_addr, type_name);
            }
            else
            {
                uart_manager_print("Failed to send SET_NODE_TYPE\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "token") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: ota_config.set.token <hex_token>\r\n");
                return;
            }

            uint32_t token = (uint32_t)strtoul(args, NULL, 0);
            ota_config_set_auth_token(token);
            uart_manager_print("Auth token set to: 0x%08X\r\n", (unsigned int)token);
        }
        else if (strcmp(target, "gpio") == 0)
        {
            if (args == NULL || *args == '\0')
            {
                uart_manager_print("Usage: ota_config.set.gpio <target_addr> <pin> <state>\r\n");
                uart_manager_print("  target_addr: Target node address (hex: 0xABCD)\r\n");
                uart_manager_print("  pin: 0=LED_GREEN\r\n");
                uart_manager_print("  state: 0=OFF, 1=ON\r\n");
                uart_manager_print("Example: ota_config.set.gpio 0x0002 0 1\r\n");
                return;
            }

            char* endptr;
            uint16_t target_addr = (uint16_t)strtoul(args, &endptr, 0);

            if (endptr == args)
            {
                uart_manager_print("ERR: Invalid target address\r\n");
                return;
            }

            // Skip whitespace
            while (*endptr == ' ' || *endptr == '\t')
                endptr++;

            uint8_t pin = (uint8_t)strtoul(endptr, &endptr, 0);

            // Skip whitespace
            while (*endptr == ' ' || *endptr == '\t')
                endptr++;

            uint8_t state = (uint8_t)strtoul(endptr, NULL, 0);

            if (ota_config_send_set_gpio(target_addr, pin, state))
            {
                uart_manager_print("Sent SET_GPIO to 0x%04X: pin=%d, state=%d\r\n", target_addr,
                                   pin, state);
            }
            else
            {
                uart_manager_print("Failed to send SET_GPIO\r\n");
            }
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "token") == 0)
        {
            uint32_t token = ota_config_get_auth_token();
            uart_manager_print("Auth token: 0x%08X\r\n", (unsigned int)token);
        }
        else if (strcmp(target, "stats") == 0)
        {
            uint32_t requests_sent, responses_received, auth_failures;
            ota_config_get_stats(&requests_sent, &responses_received, &auth_failures);
            uart_manager_print("OTA Config Statistics:\r\n");
            uart_manager_print("  Requests sent: %lu\r\n", (unsigned long)requests_sent);
            uart_manager_print("  Responses received: %lu\r\n", (unsigned long)responses_received);
            uart_manager_print("  Auth failures: %lu\r\n", (unsigned long)auth_failures);
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

STATIC void uart_cmd_router_handle_twr(const char* action, const char* target, const char* args)
{
    if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "address") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Address argument required (hex: 0xABCD)\r\n");
                return;
            }

            uint16_t address = (uint16_t)strtoul(args, NULL, 0);

            // Set address directly in UWB module (single source of truth)
            uwb_set_address(address, uwb_get_pan_id());

            uart_manager_print("Address set to: 0x%04X\r\n", address);
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
            uint16_t addr = uwb_get_address();
            uart_manager_print("Ranging Status:\r\n");
            uart_manager_print("  Address: 0x%04X\r\n", addr);
            uart_manager_print("\r\n");

            // Show initiator status
            initiator_status_t initiator_status;
            initiator_get_status(&initiator_status);
            uart_manager_print("  Initiator:\r\n");
            uart_manager_print("    State: %d\r\n", initiator_status.state);
            uart_manager_print("    Successful: %lu\r\n",
                               (unsigned long)initiator_status.successful_ranges);
            uart_manager_print("    Failed: %lu\r\n",
                               (unsigned long)initiator_status.failed_ranges);
            uart_manager_print("\r\n");

            // Show responder status
            responder_status_t responder_status;
            responder_get_status(&responder_status);
            uart_manager_print("  Responder:\r\n");
            uart_manager_print("    State: %d\r\n", responder_status.state);
            uart_manager_print("    Polls Received: %lu\r\n",
                               (unsigned long)responder_status.polls_received);
            uart_manager_print("    Responses Sent: %lu\r\n",
                               (unsigned long)responder_status.responses_sent);
        }
        else if (strcmp(target, "result") == 0)
        {
            twr_result_t result;
            if (initiator_get_last_result(&result))
            {
                uart_manager_print("Last Range: %.2f m, RSSI: %.1f dBm\r\n", result.distance_m,
                                   result.rssi_dbm);
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

            uint16_t target_addr = (uint16_t)strtoul(args, NULL, 0);

            if (!initiator_start_ranging(target_addr))
            {
                uart_manager_print("ERR: Failed to start ranging (check mode and UWB status)\r\n");
            }
            else
            {
                uart_manager_print("Ranging started to target 0x%04X\r\n", target_addr);
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

STATIC void uart_cmd_router_handle_system(const char* action, const char* target, const char* args)
{
    (void)args;

    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "uuid") == 0)
        {
            uint32_t word0 = platform_system_get_uuid_word(0);
            uint32_t word1 = platform_system_get_uuid_word(1);
            uint32_t word2 = platform_system_get_uuid_word(2);

            uart_manager_print("\r\n");
            uart_manager_print("=== STM32 Unique ID ===\r\n");
            uart_manager_print("Full UUID: %08lX-%08lX-%08lX\r\n", (unsigned long)word0,
                               (unsigned long)word1, (unsigned long)word2);
            uart_manager_print("\r\n");
            uart_manager_print("Copy this line to config/device_mapping.c:\r\n");
            uart_manager_print("{.uuid_word0 = 0x%08lX, .uuid_word1 = 0x%08lX, .uuid_word2 = "
                               "0x%08lX, .uwb_address = 0x????, .device_name = \"NAME\"},\r\n",
                               (unsigned long)word0, (unsigned long)word1, (unsigned long)word2);
            uart_manager_print("\r\n");
        }
        else if (strcmp(target, "info") == 0)
        {
            platform_system_device_info_t dev_info;
            if (platform_system_device_get_info(&dev_info))
            {
                uart_manager_print("\r\n");
                uart_manager_print("=== Device Information ===\r\n");
                uart_manager_print(
                    "UUID: %08lX-%08lX-%08lX\r\n", (unsigned long)dev_info.uuid_word0,
                    (unsigned long)dev_info.uuid_word1, (unsigned long)dev_info.uuid_word2);
                uart_manager_print("Known Device: %s\r\n", dev_info.is_known_device ? "Yes" : "No");
                if (dev_info.is_known_device)
                {
                    uart_manager_print("Device Name: %s\r\n", dev_info.device_name);
                    uart_manager_print("Assigned Address: 0x%04X\r\n", dev_info.assigned_address);
                }
                uart_manager_print("\r\n");
            }
            else
            {
                uart_manager_print("ERR: Device ID not initialized\r\n");
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

STATIC void uart_cmd_router_handle_sensor_fusion(const char* action, const char* target,
                                                 const char* args)
{
    if (strcmp(action, "get") == 0)
    {
        if (strcmp(target, "debug") == 0)
        {
            bool enabled = sensor_fusion_get_debug_prints_enabled();
            uart_manager_print("Sensor Fusion debug prints: %s\r\n",
                               enabled ? "ENABLED" : "DISABLED");
        }
        else if (strcmp(target, "status") == 0)
        {
            sensor_fusion_position_t pos;
            if (sensor_fusion_get_position(&pos))
            {
                uart_manager_print("\r\nSensor Fusion Status:\r\n");
                uart_manager_print("Position: (%.2f, %.2f, %.2f) m\r\n", pos.x, pos.y, pos.z);
                uart_manager_print("Velocity: (%.2f, %.2f, %.2f) m/s\r\n", pos.vx, pos.vy, pos.vz);
                uart_manager_print("Confidence: %.2f\r\n", pos.confidence);
                uart_manager_print("Valid: YES\r\n");
            }
            else
            {
                uart_manager_print("\r\nSensor Fusion Status:\r\n");
                uart_manager_print("Valid: NO\r\n");
            }
        }
        else if (strcmp(target, "active") == 0)
        {
            bool active = sensor_fusion_is_active();
            uart_manager_print("Sensor Fusion: %s\r\n", active ? "ACTIVE" : "STOPPED");
        }
        else if (strcmp(target, "imu") == 0)
        {
            bool enabled = sensor_fusion_get_imu_enabled();
            uart_manager_print("IMU integration: %s\r\n", enabled ? "ENABLED" : "DISABLED");
        }
        else if (strcmp(target, "noise") == 0)
        {
            float pos, vel, att;
            sensor_fusion_get_process_noise(&pos, &vel, &att);
            uart_manager_print("Process Noise:\r\n");
            uart_manager_print("  Position: %.4f\r\n", pos);
            uart_manager_print("  Velocity: %.4f\r\n", vel);
            uart_manager_print("  Attitude: %.4f\r\n", att);
        }
        else if (strcmp(target, "config") == 0)
        {
            kalmanCoreParams_t params;
            sensor_fusion_get_kalman_params(&params);
            uart_manager_print("\r\nKalman Filter Configuration:\r\n");
            uart_manager_print("Initial Variances:\r\n");
            uart_manager_print("  Position XY: %.4f\r\n", params.stdDevInitialPosition_xy);
            uart_manager_print("  Position Z: %.4f\r\n", params.stdDevInitialPosition_z);
            uart_manager_print("  Velocity: %.4f\r\n", params.stdDevInitialVelocity);
            uart_manager_print("  Attitude R/P: %.4f\r\n", params.stdDevInitialAttitude_rollpitch);
            uart_manager_print("  Attitude Yaw: %.4f\r\n", params.stdDevInitialAttitude_yaw);
            uart_manager_print("Process Noise:\r\n");
            uart_manager_print("  Accel XY: %.4f\r\n", params.procNoiseAcc_xy);
            uart_manager_print("  Accel Z: %.4f\r\n", params.procNoiseAcc_z);
            uart_manager_print("  Velocity: %.4f\r\n", params.procNoiseVel);
            uart_manager_print("  Position: %.4f\r\n", params.procNoisePos);
            uart_manager_print("  Attitude: %.4f\r\n", params.procNoiseAtt);
        }
        else
        {
            uart_manager_print("ERR: Unknown target '%s'\r\n", target);
        }
    }
    else if (strcmp(action, "set") == 0)
    {
        if (strcmp(target, "debug") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Missing argument (0=disable, 1=enable)\r\n");
                return;
            }

            int enable = atoi(args);
            if (enable != 0 && enable != 1)
            {
                uart_manager_print("ERR: Invalid value. Use 0 (disable) or 1 (enable)\r\n");
                return;
            }

            sensor_fusion_enable_debug_prints(enable != 0);
            uart_manager_print("Sensor Fusion debug prints %s\r\n",
                               enable ? "ENABLED" : "DISABLED");
        }
        else if (strcmp(target, "active") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Missing argument (0=stop, 1=start)\r\n");
                return;
            }

            int enable = atoi(args);
            if (enable != 0 && enable != 1)
            {
                uart_manager_print("ERR: Invalid value. Use 0 (stop) or 1 (start)\r\n");
                return;
            }

            if (enable)
            {
                sensor_fusion_start();
                uart_manager_print("Sensor Fusion STARTED (data cleared)\r\n");
            }
            else
            {
                sensor_fusion_stop();
                uart_manager_print("Sensor Fusion STOPPED\r\n");
            }
        }
        else if (strcmp(target, "imu") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Missing argument (0=disable, 1=enable)\r\n");
                return;
            }
            
            int enable = atoi(args);
            if (enable != 0 && enable != 1)
            {
                uart_manager_print("ERR: Invalid value. Use 0 (disable) or 1 (enable)\r\n");
                return;
            }
            
            sensor_fusion_enable_imu(enable != 0);
            uart_manager_print("IMU integration %s\r\n", enable ? "ENABLED" : "DISABLED");
        }
        else if (strcmp(target, "noise") == 0)
        {
            if (args == NULL)
            {
                uart_manager_print("ERR: Missing arguments (pos vel att)\r\n");
                uart_manager_print("Example: sf.set.noise 0.01 0.1 0.001\r\n");
                return;
            }
            
            char* end_ptr;
            float pos = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid position noise\r\n");
                return;
            }
            
            args = skip_whitespace(end_ptr);
            float vel = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid velocity noise\r\n");
                return;
            }
            
            args = skip_whitespace(end_ptr);
            float att = strtof(args, &end_ptr);
            if (end_ptr == args)
            {
                uart_manager_print("ERR: Invalid attitude noise\r\n");
                return;
            }
            
            sensor_fusion_set_process_noise(pos, vel, att);
            uart_manager_print("Process noise updated: pos=%.4f, vel=%.4f, att=%.4f\r\n", pos, vel, att);
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
    char* dot1   = strchr(module, '.');
    if (dot1 == NULL)
    {
        uart_manager_print("ERR: Invalid format. Use module.action.target\r\n");
        return;
    }
    *dot1        = '\0';
    char* action = dot1 + 1;

    char* dot2 = strchr(action, '.');
    if (dot2 == NULL)
    {
        uart_manager_print("ERR: Invalid format. Use module.action.target\r\n");
        return;
    }
    *dot2        = '\0';
    char* target = dot2 + 1;

    char* args  = NULL;
    char* space = strchr(target, ' ');
    if (space != NULL)
    {
        *space = '\0';
        args   = (char*)skip_whitespace(space + 1);
    }

    if (strcmp(module, "beacon") == 0)
    {
        uart_cmd_router_handle_beacon(action, target, args);
    }
    else if (strcmp(module, "imu") == 0)
    {
        uart_cmd_router_handle_imu(action, target, args);
    }
    else if (strcmp(module, "uwb_node") == 0)
    {
        uart_cmd_router_handle_node(action, target, args);
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
    else if (strcmp(module, "twr") == 0)
    {
        uart_cmd_router_handle_twr(action, target, args);
    }
    else if (strcmp(module, "twrmgr") == 0)
    {
        uart_cmd_router_handle_twr_manager(action, target, args);
    }
    else if (strcmp(module, "stopwatch") == 0)
    {
        uart_cmd_router_handle_stopwatch(action, target, args);
    }
    else if (strcmp(module, "ota_config") == 0)
    {
        uart_cmd_router_handle_ota_config(action, target, args);
    }
    else if (strcmp(module, "system") == 0)
    {
        uart_cmd_router_handle_system(action, target, args);
    }
    else if (strcmp(module, "sf") == 0)
    {
        uart_cmd_router_handle_sensor_fusion(action, target, args);
    }
    else
    {
        uart_manager_print("ERR: Unknown module '%s'\r\n", module);
    }
}
