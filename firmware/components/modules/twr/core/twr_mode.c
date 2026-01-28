/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr_mode.h"
#include "FreeRTOS.h"
#include "../roles/anchor/anchor.h"
#include "../roles/tag/tag.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC twr_mode_status_t mode_status = {.current_mode       = TWR_MODE_DISABLED,
                                        .requested_mode     = TWR_MODE_DISABLED,
                                        .requester          = NULL,
                                        .transition_pending = false,
                                        .mode_changes       = 0};

STATIC bool mode_manager_initialized = false;

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void twr_mode_init(void)
{
    if (mode_manager_initialized)
    {
        return;
    }

    mode_status.current_mode       = TWR_MODE_DISABLED;
    mode_status.requested_mode     = TWR_MODE_DISABLED;
    mode_status.requester          = NULL;
    mode_status.transition_pending = false;
    mode_status.mode_changes       = 0;

    mode_manager_initialized = true;
}

bool twr_mode_request(twr_mode_e new_mode, const char* requester)
{
    if (!mode_manager_initialized)
    {
        return false;
    }

    // Don't queue multiple transitions - take the latest request
    mode_status.requested_mode     = new_mode;
    mode_status.requester          = requester;
    mode_status.transition_pending = (new_mode != mode_status.current_mode);

    return true;
}

twr_mode_e twr_mode_get_current(void)
{
    return mode_status.current_mode;
}

bool twr_mode_is_transitioning(void)
{
    return mode_status.transition_pending;
}

void twr_mode_process(void)
{
    if (!mode_manager_initialized || !mode_status.transition_pending)
    {
        return;
    }

    twr_mode_e old_mode = mode_status.current_mode;
    twr_mode_e new_mode = mode_status.requested_mode;

    // Step 1: Stop current mode
    switch (old_mode)
    {
        case TWR_MODE_TAG:
            tag_stop();
            break;
        case TWR_MODE_ANCHOR:
            anchor_stop();
            break;
        case TWR_MODE_DISABLED:
            // Nothing to stop
            break;
    }

    // Step 2: Update mode status first so start functions see correct mode
    mode_status.current_mode = new_mode;

    // Step 3: Start new mode
    bool success = true;
    switch (new_mode)
    {
        case TWR_MODE_TAG:
            tag_init(); // Reinitialize context
            success = tag_start();
            break;
        case TWR_MODE_ANCHOR:
            anchor_init(); // Reinitialize context
            success = anchor_start();
            break;
        case TWR_MODE_DISABLED:
            // Nothing to start
            break;
    }

    // Step 4: Handle success/failure
    if (success)
    {
        mode_status.mode_changes++;
    }
    else
    {
        // Failure - revert to disabled
        mode_status.current_mode   = TWR_MODE_DISABLED;
        mode_status.requested_mode = TWR_MODE_DISABLED;
    }

    mode_status.transition_pending = false;
}

void twr_mode_get_status(twr_mode_status_t* status)
{
    if (status != NULL)
    {
        *status = mode_status;
    }
}