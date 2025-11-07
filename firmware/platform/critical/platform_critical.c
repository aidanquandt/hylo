/*---------------------------------------------------------------------------
 * @file    platform_critical.c
 * @brief   Platform abstraction for critical sections - FreeRTOS implementation
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_critical.h"
#include "FreeRTOS.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

platform_critical_state_t platform_critical_enter(void)
{
    // Enter FreeRTOS critical section
    // This disables interrupts and prevents task switching
    taskENTER_CRITICAL();
    
    // FreeRTOS critical sections don't return a state value,
    // but we return 0 to maintain interface consistency
    return 0;
}

void platform_critical_exit(platform_critical_state_t state)
{
    // Exit FreeRTOS critical section
    // This restores interrupts and re-enables task switching
    (void)state;  // State not used with FreeRTOS
    taskEXIT_CRITICAL();
}
