#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "twr_state_machine.h"

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void twr_init(void);
void twr_rx_callback(twr_context_t* ctx, const uint8_t* data, uint16_t length, uint16_t src_addr,
                     uint64_t rx_timestamp);
void twr_tx_complete_callback(twr_context_t* ctx, uint32_t message_id, uint64_t tx_timestamp);
bool twr_start_ranging(uint16_t peer_addr);
void twr_stop(void);
bool twr_is_active(const twr_context_t* ctx);
void twr_cancel(twr_context_t* ctx);