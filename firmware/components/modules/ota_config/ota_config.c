/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "ota_config.h"
#include "FreeRTOS.h"
#include "backoff.h"
#include "error_handler.h"
#include "module.h"
#include "platform_gpio.h"
#include "semphr.h"
#include "task.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define OTA_CONFIG_MAX_RETRIES (5U)           // Max retry attempts
#define OTA_CONFIG_RESPONSE_TIMEOUT_MS (200U) // Wait 200ms for response
#define MAX_PENDING_REQUESTS (4U)             // Concurrent request slots
#define PAN_ID_KEEP_CURRENT (0xFFFFU)         // Keep existing PAN ID
#define INVALID_ADDRESS_MIN (0x0000U)         // Reserved address
#define INVALID_ADDRESS_MAX (0xFFFFU)         // Broadcast address

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    bool in_use;                         // Slot occupied
    TaskHandle_t waiting_task;           // Task waiting for response
    uint16_t target_addr;                // Target node address (old address for SET_ADDRESS)
    uint16_t new_addr;                   // New address (for SET_ADDRESS only, 0 otherwise)
    uint16_t sequence;                   // Message sequence number
    ota_config_status_e response_status; // Response status from target
} ota_config_pending_request_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uint32_t auth_token                              = OTA_CONFIG_DEFAULT_AUTH_TOKEN;
STATIC ota_config_response_callback_t response_callback = NULL;
STATIC bool module_initialized                          = false;

// Pending request table for concurrent operations
STATIC ota_config_pending_request_t pending_requests[MAX_PENDING_REQUESTS];
STATIC SemaphoreHandle_t pending_requests_mutex = NULL;

STATIC struct
{
    uint32_t requests_sent;
    uint32_t responses_received;
    uint32_t auth_failures;
    uint32_t invalid_messages;
    uint32_t retries_performed;
    uint32_t retry_successes;
} ota_config_stats = {0};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void ota_config_module_init(void);
STATIC void ota_config_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                        uint64_t rx_timestamp);
STATIC void ota_config_handle_set_address(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void ota_config_handle_set_position(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void ota_config_handle_set_node_type(const uint8_t* data, uint16_t length,
                                            uint16_t src_addr);
STATIC void ota_config_handle_set_gpio(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void ota_config_handle_response(const uint8_t* data, uint16_t length, uint16_t src_addr);
STATIC void ota_config_send_response(uint16_t dest_addr, uint8_t status, uint16_t sequence);
STATIC int8_t ota_config_allocate_slot(uint16_t target_addr, uint16_t sequence);
STATIC void ota_config_free_slot(int8_t slot);
STATIC bool ota_config_send_with_retry(uint16_t target_addr, const uint8_t* msg_data,
                                       uint16_t msg_size, uint16_t sequence, const char* msg_name);
STATIC bool ota_config_send_with_retry_ex(uint16_t target_addr, uint16_t new_addr,
                                          const uint8_t* msg_data, uint16_t msg_size,
                                          uint16_t sequence, const char* msg_name);
STATIC bool ota_config_verify_auth(uint32_t received_token, uint16_t src_addr, uint16_t sequence);
STATIC void ota_config_init_message_header(protocol_header_t* header, uint8_t msg_type);

/*---------------------------------------------------------------------------
 * Module Registration
 *---------------------------------------------------------------------------*/
extern const module_S ota_config_module;

const module_S ota_config_module = {
    .module_name         = "ota_config",
    .module_init         = ota_config_module_init,
    .module_process_1Hz  = NULL,
    .module_process_10Hz = NULL,
};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void ota_config_module_init(void)
{
    // Initialize pending request table
    memset(pending_requests, 0, sizeof(pending_requests));

    // Create mutex for pending request table access
    pending_requests_mutex = xSemaphoreCreateMutex();
    if (pending_requests_mutex == NULL)
    {
        return;
    }

    // Register protocol handler for OTA_CONFIG messages
    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_OTA_CONFIG, ota_config_protocol_handler))
    {
        return;
    }

    module_initialized = true;
}

STATIC void ota_config_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                        uint64_t rx_timestamp)
{
    (void)rx_timestamp; // Unused

    if (data == NULL || length < sizeof(protocol_header_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_header_t* header = (const protocol_header_t*)data;

    // Route to appropriate handler based on message type
    switch (header->msg_type)
    {
        case OTA_CONFIG_MSG_SET_ADDRESS:
            ota_config_handle_set_address(data, length, src_addr);
            break;

        case OTA_CONFIG_MSG_SET_POSITION:
            ota_config_handle_set_position(data, length, src_addr);
            break;

        case OTA_CONFIG_MSG_SET_UWB_NODE_TYPE:
            ota_config_handle_set_node_type(data, length, src_addr);
            break;

        case OTA_CONFIG_MSG_SET_GPIO:
            ota_config_handle_set_gpio(data, length, src_addr);
            break;

        case OTA_CONFIG_MSG_RESPONSE:
            ota_config_handle_response(data, length, src_addr);
            break;

        default:
            ota_config_stats.invalid_messages++;
            error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Unknown message type: 0x%02X",
                              header->msg_type);
            break;
    }
}

STATIC void ota_config_handle_set_address(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    if (length < sizeof(protocol_ota_config_set_address_msg_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_ota_config_set_address_msg_t* msg =
        (const protocol_ota_config_set_address_msg_t*)data;

    // Verify authentication token
    if (!ota_config_verify_auth(msg->auth_token, src_addr, msg->header.sequence))
    {
        return;
    }

    // Validate parameters
    if (msg->new_address == INVALID_ADDRESS_MIN || msg->new_address == INVALID_ADDRESS_MAX)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Invalid address: 0x%04X",
                          msg->new_address);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_INVALID_PARAM, msg->header.sequence);
        return;
    }

    // Apply new address
    uint16_t pan_id = (msg->new_pan_id == PAN_ID_KEEP_CURRENT) ? uwb_get_pan_id() : msg->new_pan_id;
    uwb_set_address(msg->new_address, pan_id);

    error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                      "Address changed: 0x%04X -> 0x%04X, PAN: 0x%04X", src_addr, msg->new_address,
                      pan_id);

    ota_config_send_response(src_addr, OTA_CONFIG_STATUS_SUCCESS, msg->header.sequence);
}

STATIC void ota_config_handle_set_position(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    if (length < sizeof(protocol_ota_config_set_position_msg_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_ota_config_set_position_msg_t* msg =
        (const protocol_ota_config_set_position_msg_t*)data;

    // Verify authentication token
    if (!ota_config_verify_auth(msg->auth_token, src_addr, msg->header.sequence))
    {
        return;
    }

    // Apply new position (copy to avoid unaligned pointer)
    vec3_t position = msg->position;
    uwb_node_set_position(&position);

    error_handler_log(ERROR_SEVERITY_INFO, "ota_config", "Position set: (%.2f, %.2f, %.2f)",
                      msg->position.x, msg->position.y, msg->position.z);

    ota_config_send_response(src_addr, OTA_CONFIG_STATUS_SUCCESS, msg->header.sequence);
}

STATIC void ota_config_handle_set_node_type(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    if (length < sizeof(protocol_ota_config_set_node_type_msg_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_ota_config_set_node_type_msg_t* msg =
        (const protocol_ota_config_set_node_type_msg_t*)data;

    // Verify authentication token
    if (!ota_config_verify_auth(msg->auth_token, src_addr, msg->header.sequence))
    {
        return;
    }

    // Validate node type
    if (msg->uwb_node_type > UWB_NODE_TYPE_HYBRID)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Invalid node type: %d",
                          msg->uwb_node_type);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_INVALID_PARAM, msg->header.sequence);
        return;
    }

    // Apply new node type
    uwb_node_set_type((uwb_node_type_e)msg->uwb_node_type);

    error_handler_log(ERROR_SEVERITY_INFO, "ota_config", "Node type set: %d", msg->uwb_node_type);

    ota_config_send_response(src_addr, OTA_CONFIG_STATUS_SUCCESS, msg->header.sequence);
}

STATIC void ota_config_handle_set_gpio(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    if (length < sizeof(protocol_ota_config_set_gpio_msg_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_ota_config_set_gpio_msg_t* msg = (const protocol_ota_config_set_gpio_msg_t*)data;

    // Verify authentication token
    if (!ota_config_verify_auth(msg->auth_token, src_addr, msg->header.sequence))
    {
        return;
    }

    // Validate pin (currently only LED_GREEN supported)
    if (msg->pin != 0)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Invalid GPIO pin: %d", msg->pin);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_INVALID_PARAM, msg->header.sequence);
        return;
    }

    // Validate state
    if (msg->state > 1)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Invalid GPIO state: %d",
                          msg->state);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_INVALID_PARAM, msg->header.sequence);
        return;
    }

    // Apply GPIO state
    platform_gpio_set_led_green((platform_gpio_state_t)msg->state);

    error_handler_log(ERROR_SEVERITY_INFO, "ota_config", "GPIO set: pin=%d, state=%d", msg->pin,
                      msg->state);

    ota_config_send_response(src_addr, OTA_CONFIG_STATUS_SUCCESS, msg->header.sequence);
}

STATIC void ota_config_handle_response(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    if (length < sizeof(protocol_ota_config_response_msg_t))
    {
        ota_config_stats.invalid_messages++;
        return;
    }

    const protocol_ota_config_response_msg_t* msg = (const protocol_ota_config_response_msg_t*)data;
    ota_config_stats.responses_received++;

    error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                      "Response from 0x%04X: status=%d, addr=0x%04X, type=%d", src_addr,
                      msg->status, msg->current_address, msg->uwb_node_type);

    // Find matching pending request and notify waiting task
    if (xSemaphoreTake(pending_requests_mutex, portMAX_DELAY) == pdTRUE)
    {
        for (uint8_t i = 0; i < MAX_PENDING_REQUESTS; i++)
        {
            if (pending_requests[i].in_use && pending_requests[i].sequence == msg->header.sequence)
            {
                // For SET_ADDRESS, response comes from NEW address (after change)
                // For others, response comes from target_addr
                bool addr_match =
                    (pending_requests[i].target_addr == src_addr) ||
                    (pending_requests[i].new_addr != 0 && pending_requests[i].new_addr == src_addr);

                if (addr_match)
                {
                    pending_requests[i].response_status = (ota_config_status_e)msg->status;
                    xTaskNotifyGive(pending_requests[i].waiting_task);
                    break;
                }
            }
        }
        xSemaphoreGive(pending_requests_mutex);
    }

    // Notify callback if registered
    if (response_callback != NULL)
    {
        ota_config_result_t result = {.success     = (msg->status == OTA_CONFIG_STATUS_SUCCESS),
                                      .status_code = msg->status,
                                      .response_received = true};
        response_callback(src_addr, result);
    }
}

STATIC void ota_config_send_response(uint16_t dest_addr, uint8_t status, uint16_t sequence)
{
    protocol_ota_config_response_msg_t response = {0};

    // Fill header
    response.header.protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    response.header.msg_type      = OTA_CONFIG_MSG_RESPONSE;
    response.header.sequence      = sequence;

    // Fill status and current configuration
    response.status          = status;
    response.current_address = uwb_get_address();
    response.current_pan_id  = uwb_get_pan_id();
    response.uwb_node_type   = (uint8_t)uwb_node_get_type();

    vec3_t position;
    response.position_known = uwb_node_get_position(&position);
    if (response.position_known)
    {
        response.position = position;
    }

    // Send response
    uwb_send_message((const uint8_t*)&response, sizeof(response), dest_addr);
}

/*---------------------------------------------------------------------------
 * Helper Functions for Pending Request Management
 *---------------------------------------------------------------------------*/
STATIC bool ota_config_verify_auth(uint32_t received_token, uint16_t src_addr, uint16_t sequence)
{
    if (received_token != auth_token)
    {
        ota_config_stats.auth_failures++;
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config",
                          "Auth failed from 0x%04X (got 0x%08X, expected 0x%08X)", src_addr,
                          (unsigned int)received_token, (unsigned int)auth_token);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_AUTH_FAILED, sequence);
        return false;
    }
    return true;
}

STATIC void ota_config_init_message_header(protocol_header_t* header, uint8_t msg_type)
{
    header->protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    header->msg_type      = msg_type;
    header->sequence      = (uint16_t)(ota_config_stats.requests_sent & 0xFFFF);
}

STATIC int8_t ota_config_allocate_slot(uint16_t target_addr, uint16_t sequence)
{
    int8_t slot = -1;

    if (xSemaphoreTake(pending_requests_mutex, portMAX_DELAY) == pdTRUE)
    {
        // Find free slot
        for (uint8_t i = 0; i < MAX_PENDING_REQUESTS; i++)
        {
            if (!pending_requests[i].in_use)
            {
                pending_requests[i].in_use          = true;
                pending_requests[i].waiting_task    = xTaskGetCurrentTaskHandle();
                pending_requests[i].target_addr     = target_addr;
                pending_requests[i].new_addr        = 0; // Default: no address change
                pending_requests[i].sequence        = sequence;
                pending_requests[i].response_status = OTA_CONFIG_STATUS_OPERATION_FAILED;
                slot                                = (int8_t)i;
                break;
            }
        }
        xSemaphoreGive(pending_requests_mutex);
    }

    return slot;
}

STATIC void ota_config_free_slot(int8_t slot)
{
    if (slot < 0 || slot >= MAX_PENDING_REQUESTS)
    {
        return;
    }

    if (xSemaphoreTake(pending_requests_mutex, portMAX_DELAY) == pdTRUE)
    {
        pending_requests[slot].in_use = false;
        xSemaphoreGive(pending_requests_mutex);
    }
}

STATIC bool ota_config_send_with_retry(uint16_t target_addr, const uint8_t* msg_data,
                                       uint16_t msg_size, uint16_t sequence, const char* msg_name)
{
    return ota_config_send_with_retry_ex(target_addr, 0, msg_data, msg_size, sequence, msg_name);
}

STATIC bool ota_config_send_with_retry_ex(uint16_t target_addr, uint16_t new_addr,
                                          const uint8_t* msg_data, uint16_t msg_size,
                                          uint16_t sequence, const char* msg_name)
{
    for (uint8_t attempt = 0; attempt <= OTA_CONFIG_MAX_RETRIES; attempt++)
    {
        if (attempt > 0)
        {
            uint32_t delay_ms = backoff_calculate_simple(attempt, 50, 500, true);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        // Allocate slot for this request
        int8_t slot = ota_config_allocate_slot(target_addr, sequence);
        if (slot < 0)
        {
            continue; // No slots available, retry after backoff
        }

        // Store new_addr if this is a SET_ADDRESS command
        if (new_addr != 0)
        {
            if (xSemaphoreTake(pending_requests_mutex, portMAX_DELAY) == pdTRUE)
            {
                pending_requests[slot].new_addr = new_addr;
                xSemaphoreGive(pending_requests_mutex);
            }
        }

        // Send message
        uwb_send_result_t result = uwb_send_message(msg_data, msg_size, target_addr);
        if (!result.success)
        {
            ota_config_free_slot(slot);
            continue; // Queue full, retry after backoff
        }

        // Wait for response notification
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(OTA_CONFIG_RESPONSE_TIMEOUT_MS));
        bool success =
            (notified > 0) && (pending_requests[slot].response_status == OTA_CONFIG_STATUS_SUCCESS);

        ota_config_free_slot(slot);

        if (success)
        {
            ota_config_stats.requests_sent++;
            if (attempt > 0)
            {
                ota_config_stats.retries_performed += attempt;
                ota_config_stats.retry_successes++;
            }
            // For SET_ADDRESS, show the new address in the log
            uint16_t response_addr = (new_addr != 0) ? new_addr : target_addr;
            error_handler_log(ERROR_SEVERITY_INFO, "ota_config", "%s ACK from 0x%04X (attempts=%u)",
                              msg_name, response_addr, attempt + 1);
            return true;
        }
    }

    error_handler_log(ERROR_SEVERITY_WARNING, "ota_config",
                      "%s no ACK from 0x%04X after %u attempts", msg_name, target_addr,
                      OTA_CONFIG_MAX_RETRIES + 1);
    return false;
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
void ota_config_set_auth_token(uint32_t token)
{
    auth_token = token;
    error_handler_log(ERROR_SEVERITY_INFO, "ota_config", "Auth token set: 0x%08X",
                      (unsigned int)token);
}

uint32_t ota_config_get_auth_token(void)
{
    return auth_token;
}

bool ota_config_send_set_address(uint16_t target_address, uint16_t new_address, uint16_t new_pan_id)
{
    if (!module_initialized || pending_requests_mutex == NULL)
    {
        return false;
    }

    protocol_ota_config_set_address_msg_t msg = {0};

    ota_config_init_message_header(&msg.header, OTA_CONFIG_MSG_SET_ADDRESS);
    msg.auth_token  = auth_token;
    msg.new_address = new_address;
    msg.new_pan_id  = new_pan_id;

    return ota_config_send_with_retry_ex(target_address, new_address, (const uint8_t*)&msg,
                                         sizeof(msg), msg.header.sequence, "SET_ADDRESS");
}

bool ota_config_send_set_position(uint16_t target_address, const vec3_t* position)
{
    if (!module_initialized || position == NULL || pending_requests_mutex == NULL)
    {
        return false;
    }

    protocol_ota_config_set_position_msg_t msg = {0};

    ota_config_init_message_header(&msg.header, OTA_CONFIG_MSG_SET_POSITION);
    msg.auth_token = auth_token;
    msg.position   = *position;

    return ota_config_send_with_retry(target_address, (const uint8_t*)&msg, sizeof(msg),
                                      msg.header.sequence, "SET_POSITION");
}

bool ota_config_send_set_node_type(uint16_t target_address, uwb_node_type_e node_type)
{
    if (!module_initialized || node_type > UWB_NODE_TYPE_HYBRID || pending_requests_mutex == NULL)
    {
        return false;
    }

    protocol_ota_config_set_node_type_msg_t msg = {0};

    ota_config_init_message_header(&msg.header, OTA_CONFIG_MSG_SET_UWB_NODE_TYPE);
    msg.auth_token    = auth_token;
    msg.uwb_node_type = (uint8_t)node_type;

    return ota_config_send_with_retry(target_address, (const uint8_t*)&msg, sizeof(msg),
                                      msg.header.sequence, "SET_NODE_TYPE");
}

bool ota_config_send_set_gpio(uint16_t target_address, uint8_t pin, uint8_t state)
{
    if (!module_initialized || pin > 0 || state > 1 || pending_requests_mutex == NULL)
    {
        return false;
    }

    protocol_ota_config_set_gpio_msg_t msg = {0};

    ota_config_init_message_header(&msg.header, OTA_CONFIG_MSG_SET_GPIO);
    msg.auth_token = auth_token;
    msg.pin        = pin;
    msg.state      = state;

    return ota_config_send_with_retry(target_address, (const uint8_t*)&msg, sizeof(msg),
                                      msg.header.sequence, "SET_GPIO");
}

void ota_config_register_response_callback(ota_config_response_callback_t callback)
{
    response_callback = callback;
}

void ota_config_get_stats(uint32_t* requests_sent, uint32_t* responses_received,
                          uint32_t* auth_failures)
{
    if (requests_sent != NULL)
    {
        *requests_sent = ota_config_stats.requests_sent;
    }
    if (responses_received != NULL)
    {
        *responses_received = ota_config_stats.responses_received;
    }
    if (auth_failures != NULL)
    {
        *auth_failures = ota_config_stats.auth_failures;
    }
}

void ota_config_reset_stats(void)
{
    ota_config_stats.requests_sent      = 0;
    ota_config_stats.responses_received = 0;
    ota_config_stats.auth_failures      = 0;
    ota_config_stats.invalid_messages   = 0;
}
