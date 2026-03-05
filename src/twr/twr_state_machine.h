#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_types.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TWR_TIMEOUT_MS 3U
#define TWR_TX_COMPLETION_TIMEOUT_MS 2U

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void twr_state_machine_init(twr_context_t* ctx, twr_role_e role, const twr_callbacks_t* callbacks,
                            void* role_context);
bool twr_state_machine_start(twr_context_t* ctx, uint16_t peer_addr);
void twr_state_machine_stop(twr_context_t* ctx);
void twr_state_machine_process(twr_context_t* ctx);
void twr_state_machine_handle_event(twr_context_t* ctx, const twr_event_t* event);
twr_state_e twr_state_machine_get_state(const twr_context_t* ctx);
void twr_state_machine_transition_to(twr_context_t* ctx, twr_state_e new_state);