#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

// Watchdog module monitors system health by running as the last module
// in each periodic task. If watchdog process functions run, the entire
// task loop is healthy.
