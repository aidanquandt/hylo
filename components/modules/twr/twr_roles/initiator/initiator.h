#pragma once

/*---------------------------------------------------------------------------
 * @file    initiator.h
 * @brief   TWR Initiator role
 * @details Initiates ranging transactions in the TWR protocol
 *---------------------------------------------------------------------------*/

#include "../twr_engine/twr_types.h"
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    INITIATOR_STATE_IDLE,
    INITIATOR_STATE_SENDING_POLL,   // Waiting for POLL TX done
    INITIATOR_STATE_WAIT_RESPONSE,  // Waiting for RESPONSE RX
    INITIATOR_STATE_SENDING_FINAL,  // Waiting for FINAL TX done
    INITIATOR_STATE_WAIT_FINAL_ACK, // Waiting for FINAL_ACK RX
    INITIATOR_STATE_PROCESS_RESULT,
    INITIATOR_STATE_FAULTED
} initiator_state_e;

typedef struct
{
    initiator_state_e state;
    uint16_t target_address;
    uint32_t successful_ranges;
    uint32_t failed_ranges;
    uint32_t timeout_count;
    twr_result_t last_result;
} initiator_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void initiator_init(void);
bool initiator_start(void);
void initiator_stop(void);
bool initiator_start_ranging(uint16_t responder_addr);
bool initiator_is_ranging(void);
bool initiator_get_last_result(twr_result_t* result);
void initiator_get_status(initiator_status_t* status);
void initiator_cancel_ranging(void);

// External context for protocol layer
extern twr_context_t initiator_twr_ctx;
