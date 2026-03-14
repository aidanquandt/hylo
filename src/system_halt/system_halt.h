#pragma once

#include "common.h"

/**
 * Halt the system after optionally sending a fatal event to the host.
 * Call from task context only. Sends SystemFatalEvent (module + message), delays
 * 100 ms, then enters critical section and infinite loop with LED toggle.
 * Does not return.
 */
void system_halt(const char* module, const char* message)
    __attribute__((noreturn));
