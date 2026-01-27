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

/**
 * @brief Initialize the TWR mode manager
 */
void twr_mode_init(void);

/**
 * @brief Request a mode change
 * @param new_mode Desired mode
 * @param requester Name of requesting component (for debugging)
 * @return true if mode change initiated, false if failed
 */
bool twr_mode_request(twr_mode_e new_mode, const char* requester);

/**
 * @brief Get current TWR mode
 * @return Current mode
 */
twr_mode_e twr_mode_get_current(void);

/**
 * @brief Check if mode change is in progress
 * @return true if transition pending
 */
bool twr_mode_is_transitioning(void);

/**
 * @brief Process mode transitions (call from 1kHz task)
 */
void twr_mode_process(void);

/**
 * @brief Get mode status for debugging
 * @param status Pointer to status structure to fill
 */
void twr_mode_get_status(twr_mode_status_t* status);