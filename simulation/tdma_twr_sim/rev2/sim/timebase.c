#include "timebase.h"

unsigned long g_sim_time_ms = 0;

void sim_advance_time_ms(unsigned long delta_ms) {
    g_sim_time_ms += delta_ms;
}
