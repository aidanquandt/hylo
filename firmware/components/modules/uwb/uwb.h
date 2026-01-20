#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "uwb_port.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UWB_STATE_OFF,
    UWB_STATE_INITIALIZATION,
    UWB_STATE_ACTIVE,
    UWB_STATE_FAULTED
} uwb_state_e;

typedef struct
{
    uwb_state_e state;
    uint32_t device_id;
    float temperature;
    float voltage;
    uint32_t fault_code;
    uint16_t my_address;
    uint16_t my_pan_id;
} uwb_status_t;

typedef struct
{
    uint32_t received;
    uint32_t rx_errors;
    uint32_t filtered;
} uwb_rx_stats_t;

typedef void (*uwb_protocol_handler_t)(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                       uint64_t rx_timestamp);
typedef void (*uwb_tx_done_handler_t)(uint64_t tx_timestamp);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void uwb_get_status(uwb_status_t* status);
void uwb_get_rx_stats(uwb_rx_stats_t* stats);
void uwb_reset_rx_stats(void);
bool uwb_is_ready(void);
void uwb_start(void);
void uwb_stop(void);
void uwb_set_address(uint16_t address, uint16_t pan_id);
uint16_t uwb_get_address(void);
bool uwb_soft_reset(void);
bool uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr);
bool uwb_send_message_delayed(const uint8_t* data, uint16_t length, uint16_t dest_addr,
                              uint32_t tx_time_dtuh);
uwb_dev_t* uwb_get_device(void);
bool uwb_register_protocol_handler(uint8_t protocol_type, uwb_protocol_handler_t handler);
void uwb_register_tx_done_handler(uwb_tx_done_handler_t handler);
void uwb_unregister_protocol_handler(uint8_t protocol_type);
void uwb_get_protocol_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid);
void uwb_reset_protocol_stats(void);
uint64_t uwb_get_last_tx_timestamp(void);
