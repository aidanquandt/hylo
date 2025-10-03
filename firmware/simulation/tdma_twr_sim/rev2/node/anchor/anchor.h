#pragma once
#include "node.h"

typedef struct
{
    uint64_t resp_proc_delay_ns; // fixed response latency
} anchor_ctx_S;

void anchor_init(node_S* n, anchor_ctx_S* ctx, uint64_t resp_proc_delay_ns);
extern const node_ops_S anchor_ops;
