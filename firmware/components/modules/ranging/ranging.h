#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "twr/twr_types.h"
#include <stdbool.h>
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    RANGING_MODE_DISABLED = 0,
    RANGING_MODE_TAG,
    RANGING_MODE_ANCHOR
} ranging_mode_e;

typedef struct
{
    ranging_mode_e mode;
    bool active;
    uint32_t successful_ranges;
    uint32_t failed_ranges;
    uint32_t messages_processed;
} ranging_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
bool ranging_set_mode(ranging_mode_e mode);
ranging_mode_e ranging_get_mode(void);
void ranging_get_status(ranging_status_t* status);
bool ranging_tag_start(uint16_t anchor_addr);
bool ranging_tag_is_active(void);
bool ranging_tag_get_result(float* distance_m, float* rssi_dbm);
void ranging_tag_cancel(void);
void ranging_anchor_set_address(uint16_t address);
uint16_t ranging_anchor_get_address(void);
uint32_t ranging_anchor_get_response_count(void);
