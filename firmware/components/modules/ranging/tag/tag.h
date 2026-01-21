#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "twr/twr_types.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    TAG_STATE_IDLE,
    TAG_STATE_SENDING_POLL,   // Waiting for POLL TX done
    TAG_STATE_WAIT_RESPONSE,  // Waiting for RESPONSE RX
    TAG_STATE_SENDING_FINAL,  // Waiting for FINAL TX done
    TAG_STATE_WAIT_FINAL_ACK, // Waiting for FINAL_ACK RX
    TAG_STATE_PROCESS_RESULT,
    TAG_STATE_FAULTED
} tag_state_e;

typedef struct
{
    tag_state_e state;
    uint16_t target_address;
    uint32_t successful_ranges;
    uint32_t failed_ranges;
    uint32_t timeout_count;
    twr_result_t last_result;
} tag_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void tag_init(void);
bool tag_start(void);
void tag_stop(void);
void tag_process_1kHz(void);
bool tag_start_ranging(uint16_t anchor_addr);
bool tag_is_ranging(void);
bool tag_get_last_result(twr_result_t* result);
void tag_get_status(tag_status_t* status);
void tag_cancel_ranging(void);
void tag_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr,
                     uint64_t rx_timestamp);
void tag_tx_done_callback(uint64_t tx_timestamp);
