#pragma once

#include <stdint.h>
#include <stddef.h>

// generate state machine structs

typedef struct
{
    void (*process)(void);                 // state process function, run periodically when in this state
    void (*onEntry)(uint16_t prevState);   // state on entry function, run once when transitioning from another state
    void (*onExit)(uint16_t nextState);    // state on exit function, run once when transitioning to another state
} state_s;

typedef struct
{
    uint16_t prev_state;
    uint16_t curr_state;
    uint16_t next_state;

    uint32_t timer;                        // state machine sub state timer, reset to 0 when transitioning

    uint16_t (*transitionLogic)(uint16_t currentState, uint32_t stateTimer);  // transition logic function pointer

    const state_s* states;                // state machine sub state action table
} state_machine_s;

void state_machine_periodic(state_machine_s* state_machine);
