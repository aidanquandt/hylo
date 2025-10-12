#pragma once
#include "common.h"
#include "messages.h"
#include "node.h"

typedef struct
{
    message_S msg;
    uint64_t arrival_ns;
} delivery_S;

typedef struct radio_S
{
    node_S* nodes[64];
    int node_count;
    delivery_S queue[512];
    int q_count;
    uint64_t now_ns;
} radio_S;

// Function prototypes
void radio_init(radio_S* r);
void radio_register(radio_S* r, node_S* n);
uint64_t radio_now(radio_S* r);
void radio_send(radio_S* r, const message_S* m); // schedules with ToF
void radio_run_until(radio_S* r, uint64_t until_ns); // delivers due messages
node_S* radio_find_node(radio_S* r, int id);
