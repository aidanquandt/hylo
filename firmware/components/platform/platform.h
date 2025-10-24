#pragma once


// todo - move this to somewhere else

#ifdef __GNUC__
#define TCM_FUNCTION __attribute__((section(".itcmram")))
#define TCM_VARIABLE __attribute__((section(".dtcmram")))
#else
#define TCM_FUNCTION
#define TCM_VARIABLE
#endif

#include <stdint.h>

uint32_t platform_gettick(void);
void platform_delay_ms(uint32_t delay_ms);
