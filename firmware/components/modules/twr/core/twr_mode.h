#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    TWR_MODE_DISABLED = 0,
    TWR_MODE_TAG,
    TWR_MODE_ANCHOR
} twr_mode_e;

typedef struct
{
    twr_mode_e current_mode;
    twr_mode_e requested_mode;
    const char* requester; // For debugging
    bool transition_pending;
    uint32_t mode_changes; // Statistics
} twr_mode_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

void twr_mode_init(void);
bool twr_mode_request(twr_mode_e new_mode, const char* requester);
twr_mode_e twr_mode_get_current(void);
bool twr_mode_is_transitioning(void);
void twr_mode_process(void);
void twr_mode_get_status(twr_mode_status_t* status);