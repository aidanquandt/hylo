/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "stopwatch.h"
#include "platform_timer.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint32_t start_time;
    uint32_t stop_time;
    bool is_running;
} stopwatch_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC stopwatch_t sw = {
    .start_time = 0,
    .stop_time = 0,
    .is_running = false,
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void stopwatch_start(void)
{
    sw.start_time = platform_get_timestamp();
    sw.is_running = true;
}

void stopwatch_stop(void)
{
    sw.stop_time = platform_get_timestamp();
    sw.is_running = false;
}

uint32_t stopwatch_elapsed_us(void)
{
    uint32_t end_time = sw.is_running ? platform_get_timestamp() : sw.stop_time;
    return platform_get_elapsed_us(sw.start_time, end_time);
}
