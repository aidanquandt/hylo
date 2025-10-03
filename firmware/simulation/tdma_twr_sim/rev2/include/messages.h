#pragma once
#include <stdint.h>

typedef enum
{
    msg_tdma_slot_start = 1, // broadcast; payload: active_tag_id
    msg_twr_poll       = 2,  // tag->anchor; payload: t1_ns
    msg_twr_resp       = 3   // anchor->tag; payload: t2_ns, t3_ns
} msg_type_E;

typedef struct
{
    msg_type_E type;
    int src_id;
    int dst_id;      // -1 = broadcast
    uint64_t tx_time_ns;
    uint64_t rx_time_ns; // filled by radio on delivery
    union
    {
        struct { int active_tag_id; } slot;
        struct { uint64_t t1_ns; } poll;
        struct { uint64_t t2_ns, t3_ns; } resp;
    } u;
} message_S;
