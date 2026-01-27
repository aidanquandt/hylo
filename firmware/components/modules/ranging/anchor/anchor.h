#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    ANCHOR_STATE_IDLE,
    ANCHOR_STATE_WAIT_POLL,         // Waiting for POLL from any tag
    ANCHOR_STATE_SENDING_RESPONSE,  // Waiting for RESPONSE TX done
    ANCHOR_STATE_WAIT_FINAL,        // Waiting for FINAL from specific tag
    ANCHOR_STATE_SENDING_FINAL_ACK, // Waiting for FINAL_ACK TX done
    ANCHOR_STATE_FAULTED
} anchor_state_e;

typedef struct
{
    anchor_state_e state;
    uint16_t my_address;
    uint32_t polls_received;
    uint32_t responses_sent;
    uint32_t response_failures;
    uint16_t last_tag_address;
} anchor_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void anchor_init(void);
bool anchor_start(void);
void anchor_stop(void);
void anchor_process_1kHz(void);
void anchor_set_address(uint16_t address);
uint16_t anchor_get_address(void);
void anchor_get_status(anchor_status_t* status);
uint32_t anchor_get_response_count(void);
void anchor_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr,
                        uint64_t rx_timestamp);
void anchor_tx_done_callback(uint32_t message_id, uint64_t tx_timestamp);