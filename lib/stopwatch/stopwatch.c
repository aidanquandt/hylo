/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "stopwatch.h"
#include "timer_driver.h"

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
STATIC stopwatch_t stopwatches[STOPWATCH_MAX_INSTANCES] = {0};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void stopwatch_start(uint8_t id)
{
    if (id >= STOPWATCH_MAX_INSTANCES)
    {
        return;
    }

    stopwatches[id].start_time = timer_driver_get_timestamp();
    stopwatches[id].is_running = true;
}

void stopwatch_stop(uint8_t id)
{
    if (id >= STOPWATCH_MAX_INSTANCES)
    {
        return;
    }

    stopwatches[id].stop_time  = timer_driver_get_timestamp();
    stopwatches[id].is_running = false;
}

uint32_t stopwatch_elapsed_us(uint8_t id)
{
    if (id >= STOPWATCH_MAX_INSTANCES)
    {
        return 0;
    }

    uint32_t end_time =
        stopwatches[id].is_running ? timer_driver_get_timestamp() : stopwatches[id].stop_time;
    return timer_driver_get_elapsed_us(stopwatches[id].start_time, end_time);
}

bool stopwatch_is_running(uint8_t id)
{
    if (id >= STOPWATCH_MAX_INSTANCES)
    {
        return false;
    }

    return stopwatches[id].is_running;
}
