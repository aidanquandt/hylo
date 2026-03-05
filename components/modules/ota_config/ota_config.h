#pragma once
/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "uwb_node.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define OTA_CONFIG_DEFAULT_AUTH_TOKEN (0xDECABEEFU)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    bool success;
    uint8_t status_code; // From ota_config_status_e
    bool response_received;
} ota_config_result_t;

typedef void (*ota_config_response_callback_t)(uint16_t target_address, ota_config_result_t result);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void ota_config_set_auth_token(uint32_t token);
uint32_t ota_config_get_auth_token(void);
bool ota_config_send_set_address(uint16_t target_address, uint16_t new_address,
                                 uint16_t new_pan_id);
bool ota_config_send_set_position(uint16_t target_address, const vec3_t* position);
bool ota_config_send_set_node_type(uint16_t target_address, uwb_node_type_e node_type);
bool ota_config_send_set_gpio(uint16_t target_address, uint8_t pin, uint8_t state);
void ota_config_register_response_callback(ota_config_response_callback_t callback);
void ota_config_get_stats(uint32_t* requests_sent, uint32_t* responses_received,
                          uint32_t* auth_failures);
void ota_config_reset_stats(void);
