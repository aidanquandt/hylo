#ifndef TIMEBASE_H
#define TIMEBASE_H

#include "node.h"

// Global simulation time (ms)
extern unsigned long g_sim_time_ms;

// Advance simulation time (ms)
void sim_advance_time_ms(unsigned long delta_ms);

#endif /* TIMEBASE_H */
