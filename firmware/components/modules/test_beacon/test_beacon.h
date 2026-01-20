#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    BEACON_MODE_RESPONDER,
    BEACON_MODE_MASTER
} beacon_mode_e;

typedef struct
{
    beacon_mode_e mode;
    uint16_t counter;
    uint32_t rx_count;
    uint32_t tx_count;
    uint16_t last_src_addr;
} beacon_status_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void test_beacon_get_status(beacon_status_t* status);
void test_beacon_set_mode(beacon_mode_e mode);
void test_beacon_set_counter(uint16_t value);
bool test_beacon_send_ping(uint16_t dest_addr);
bool test_beacon_send_ping_delayed(uint16_t dest_addr);
