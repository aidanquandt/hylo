#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Returns system tick in milliseconds
uint32_t Platform_Ticks(void);

#ifdef __cplusplus
}
#endif
