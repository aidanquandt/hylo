#pragma once

/*---------------------------------------------------------------------------
 * @file    responder.h
 * @brief   TWR Responder role
 * @details Responds to ranging requests in the TWR protocol
 *---------------------------------------------------------------------------*/

#include "twr_state_machine.h"
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    RESPONDER_STATE_IDLE,
    RESPONDER_STATE_WAIT_POLL,         // Waiting for POLL from any initiator
    RESPONDER_STATE_SENDING_RESPONSE,  // Waiting for RESPONSE TX done
    RESPONDER_STATE_WAIT_FINAL,        // Waiting for FINAL from specific initiator
    RESPONDER_STATE_SENDING_FINAL_ACK, // Waiting for FINAL_ACK TX done
    RESPONDER_STATE_FAULTED
} responder_state_e;

typedef struct
{
    responder_state_e state;
    uint16_t my_address;
    uint32_t polls_received;
    uint32_t responses_sent;
    uint32_t response_failures;
    uint16_t last_initiator_address;
} responder_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void responder_init(void);
bool responder_start(void);
void responder_stop(void);
void responder_get_status(responder_status_t* status);
uint32_t responder_get_response_count(void);

// External context for protocol layer
extern twr_context_t responder_twr_ctx;
