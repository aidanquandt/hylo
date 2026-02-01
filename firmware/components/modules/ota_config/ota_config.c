/*---------------------------------------------------------------------------
 * @file    ota_config.c
 * @brief   OTA configuration module implementation
 *---------------------------------------------------------------------------*/

#include "ota_config.h"
#include "error_handler.h"
#include "module.h"
#include "platform_gpio.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uint32_t auth_token                              = OTA_CONFIG_DEFAULT_AUTH_TOKEN;
STATIC ota_config_response_callback_t response_callback = NULL;
STATIC bool module_initialized                          = false;

STATIC struct
{
    uint32_t requests_sent;
    uint32_t responses_received;
    uint32_t auth_failures;
    uint32_t invalid_messages;
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
    if (msg->auth_token != auth_token)
    {
        ota_config_stats.auth_failures++;
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config",
                          "Auth failed from 0x%04X (got 0x%08X, expected 0x%08X)", src_addr,
                          (unsigned int)msg->auth_token, (unsigned int)auth_token);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_AUTH_FAILED, msg->header.sequence);
        return;
    }

    // Validate parameters
    if (msg->new_address == 0x0000 || msg->new_address == 0xFFFF)
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Invalid address: 0x%04X",
                          msg->new_address);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_INVALID_PARAM, msg->header.sequence);
        return;
    }

    // Apply new address
    uint16_t pan_id = (msg->new_pan_id == 0xFFFF) ? uwb_get_pan_id() : msg->new_pan_id;
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
    if (msg->auth_token != auth_token)
    {
        ota_config_stats.auth_failures++;
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Auth failed from 0x%04X",
                          src_addr);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_AUTH_FAILED, msg->header.sequence);
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
    if (msg->auth_token != auth_token)
    {
        ota_config_stats.auth_failures++;
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Auth failed from 0x%04X",
                          src_addr);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_AUTH_FAILED, msg->header.sequence);
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
    if (msg->auth_token != auth_token)
    {
        ota_config_stats.auth_failures++;
        error_handler_log(ERROR_SEVERITY_WARNING, "ota_config", "Auth failed from 0x%04X",
                          src_addr);
        ota_config_send_response(src_addr, OTA_CONFIG_STATUS_AUTH_FAILED, msg->header.sequence);
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
    if (!module_initialized)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "ota_config", "Module not initialized");
        return false;
    }

    protocol_ota_config_set_address_msg_t msg = {0};

    msg.header.protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    msg.header.msg_type      = OTA_CONFIG_MSG_SET_ADDRESS;
    msg.header.sequence      = (uint16_t)(ota_config_stats.requests_sent & 0xFFFF);
    msg.auth_token           = auth_token;
    msg.new_address          = new_address;
    msg.new_pan_id           = new_pan_id;

    uwb_send_result_t result = uwb_send_message((const uint8_t*)&msg, sizeof(msg), target_address);

    if (result.success)
    {
        ota_config_stats.requests_sent++;
        error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                          "Sent SET_ADDRESS to 0x%04X: new_addr=0x%04X, pan=0x%04X", target_address,
                          new_address, new_pan_id);
    }

    return result.success;
}

bool ota_config_send_set_position(uint16_t target_address, const vec3_t* position)
{
    if (!module_initialized || position == NULL)
    {
        return false;
    }

    protocol_ota_config_set_position_msg_t msg = {0};

    msg.header.protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    msg.header.msg_type      = OTA_CONFIG_MSG_SET_POSITION;
    msg.header.sequence      = (uint16_t)(ota_config_stats.requests_sent & 0xFFFF);
    msg.auth_token           = auth_token;
    msg.position             = *position;

    uwb_send_result_t result = uwb_send_message((const uint8_t*)&msg, sizeof(msg), target_address);

    if (result.success)
    {
        ota_config_stats.requests_sent++;
        error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                          "Sent SET_POSITION to 0x%04X: (%.2f, %.2f, %.2f)", target_address,
                          position->x, position->y, position->z);
    }

    return result.success;
}

bool ota_config_send_set_node_type(uint16_t target_address, uwb_node_type_e node_type)
{
    if (!module_initialized || node_type > UWB_NODE_TYPE_HYBRID)
    {
        return false;
    }

    protocol_ota_config_set_node_type_msg_t msg = {0};

    msg.header.protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    msg.header.msg_type      = OTA_CONFIG_MSG_SET_UWB_NODE_TYPE;
    msg.header.sequence      = (uint16_t)(ota_config_stats.requests_sent & 0xFFFF);
    msg.auth_token           = auth_token;
    msg.uwb_node_type        = (uint8_t)node_type;

    uwb_send_result_t result = uwb_send_message((const uint8_t*)&msg, sizeof(msg), target_address);

    if (result.success)
    {
        ota_config_stats.requests_sent++;
        error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                          "Sent SET_NODE_TYPE to 0x%04X: type=%d", target_address, node_type);
    }

    return result.success;
}

bool ota_config_send_set_gpio(uint16_t target_address, uint8_t pin, uint8_t state)
{
    if (!module_initialized || pin > 0 || state > 1)
    {
        return false;
    }

    protocol_ota_config_set_gpio_msg_t msg = {0};

    msg.header.protocol_type = PROTOCOL_TYPE_OTA_CONFIG;
    msg.header.msg_type      = OTA_CONFIG_MSG_SET_GPIO;
    msg.header.sequence      = (uint16_t)(ota_config_stats.requests_sent & 0xFFFF);
    msg.auth_token           = auth_token;
    msg.pin                  = pin;
    msg.state                = state;

    uwb_send_result_t result = uwb_send_message((const uint8_t*)&msg, sizeof(msg), target_address);

    if (result.success)
    {
        ota_config_stats.requests_sent++;
        error_handler_log(ERROR_SEVERITY_INFO, "ota_config",
                          "Sent SET_GPIO to 0x%04X: pin=%d, state=%d", target_address, pin, state);
    }

    return result.success;
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
