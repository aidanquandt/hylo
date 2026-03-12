/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "watchdog_driver.h"
#include "iwdg.h"

extern IWDG_HandleTypeDef hiwdg1;

void watchdog_driver_refresh(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
}
