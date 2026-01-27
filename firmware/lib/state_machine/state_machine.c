#include "state_machine.h"
#include <stdint.h>

void state_machine_periodic(state_machine_s* state_machine)
{
    if (NULL != state_machine)
    {
        if (NULL != state_machine->states)
        {
            state_machine->prev_state = state_machine->curr_state;
            state_machine->curr_state = state_machine->next_state;

            uint16_t curr_state = state_machine->curr_state;

            // transition logic
            if (NULL != state_machine->transitionLogic)
            {
                state_machine->next_state =
                    state_machine->transitionLogic(curr_state, state_machine->timer);
            }

            if (NULL != state_machine->states[curr_state].process)
            {
                state_machine->states[curr_state].process();
            }

            if (curr_state != state_machine->next_state)
            {
                // state transition
                state_machine->timer = 0;

                if (NULL != state_machine->states[curr_state].onExit)
                {
                    state_machine->states[curr_state].onExit(state_machine->next_state);
                }

                if (NULL != state_machine->states[state_machine->next_state].onEntry)
                {
                    state_machine->states[state_machine->next_state].onEntry(curr_state);
                }
            }
            else
            {
                if (state_machine->timer < UINT32_MAX)
                {
                    state_machine->timer++;
                }
            }
        }
    }
}

void state_machine_force_transition(state_machine_s* state_machine, uint16_t new_state)
{
    if (NULL != state_machine)
    {
        if (NULL != state_machine->states)
        {
            uint16_t curr_state = state_machine->curr_state;

            if (curr_state != new_state)
            {
                // Call onExit for current state
                if (NULL != state_machine->states[curr_state].onExit)
                {
                    state_machine->states[curr_state].onExit(new_state);
                }

                // Update state
                state_machine->prev_state = curr_state;
                state_machine->curr_state = new_state;
                state_machine->next_state = new_state;
                state_machine->timer      = 0;

                // Call onEntry for new state
                if (NULL != state_machine->states[new_state].onEntry)
                {
                    state_machine->states[new_state].onEntry(curr_state);
                }
            }
        }
    }
}
