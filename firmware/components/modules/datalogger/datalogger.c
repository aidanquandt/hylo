/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "datalogger.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define STATS_UPDATE_INTERVAL 50

/*---------------------------------------------------------------------------
 * Data declarations
 *---------------------------------------------------------------------------*/
STATIC uint32_t stats_counter = 0;
STATIC char stats_buffer[512];

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void datalogger_init(void) {
    stats_counter = 0;
}

void datalogger_process(void) {
    stats_counter++;
    if (stats_counter >= STATS_UPDATE_INTERVAL) {
        stats_counter = 0;
        vTaskGetRunTimeStats(stats_buffer);
    }
}