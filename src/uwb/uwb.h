#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "uwb_driver.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UWB_STATE_OFF,
    UWB_STATE_INITIALIZATION,
    UWB_STATE_ACTIVE,
    UWB_STATE_RETRY,  // Transient failure - will retry init
    UWB_STATE_FAULTED // Permanent failure - manual intervention needed
} uwb_state_e;

typedef struct
{
    uwb_state_e state;
    uint32_t device_id;
    uint32_t fault_code;
    uint16_t my_address;
    uint16_t my_pan_id;
} uwb_status_t;

typedef struct
{
    uint32_t received;
    uint32_t filtered;
} uwb_rx_stats_t;

typedef struct
{
    bool success;
    uint32_t message_id; // Unique ID to correlate with TX complete callback (never 0)
} uwb_send_result_t;

/* Driver-level statistics (app-layer copy; populated by uwb_get_driver_statistics) */
typedef struct
{
    uint32_t rx_ok_count;
    uint32_t rx_timeout_count;
    uint32_t tx_done_count;
    uint32_t send_message_count;
    uint16_t PHE;
    uint16_t RSL;
    uint16_t CRCG;
    uint16_t CRCB;
    uint8_t ARFE;
    uint8_t OVER;
    uint16_t SFDTO;
    uint16_t PTO;
    uint8_t RTO;
    uint16_t TXF;
    uint32_t irq_status;
    uint32_t status_lo;
    uint32_t status_hi;
} uwb_driver_stats_t;

typedef void (*uwb_protocol_handler_t)(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                       uint64_t rx_timestamp);
typedef void (*uwb_tx_complete_handler_t)(uint32_t message_id, uint64_t tx_timestamp);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void uwb_init(void);
void uwb_get_status(uwb_status_t* status);
void uwb_get_rx_stats(uwb_rx_stats_t* stats);
void uwb_reset_rx_stats(void);
uint32_t uwb_get_tx_queue_overflows(void);
uint32_t uwb_get_rx_queue_overflows(void);
uint32_t uwb_get_tx_timeouts(void);
uint32_t uwb_get_tx_queue_depth(void); // Number of messages waiting to be transmitted
bool uwb_is_ready(void);
void uwb_start(void);
void uwb_stop(void);
void uwb_set_address(uint16_t address, uint16_t pan_id);
uint16_t uwb_get_address(void);
uint16_t uwb_get_pan_id(void);
bool uwb_soft_reset(void);
uwb_send_result_t uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr);
bool uwb_register_protocol_handler(uint8_t protocol_type, uwb_protocol_handler_t handler);
void uwb_register_tx_complete_handler(uwb_tx_complete_handler_t handler);
void uwb_unregister_protocol_handler(uint8_t protocol_type);
void uwb_get_protocol_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid);
void uwb_reset_protocol_stats(void);
void uwb_get_driver_statistics(uwb_driver_stats_t* out);
