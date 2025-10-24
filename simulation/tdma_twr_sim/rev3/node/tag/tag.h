#pragma once
#include "node.h"

typedef enum
{
    TAG_IDLE = 0,
    TAG_WAIT_RESP = 1
} tag_state_E;

typedef struct
{
    int anchors[32];
    int num_anchors;
    int idx; // current anchor index during slot
    tag_state_E st;
    uint64_t t1_ns; // last POLL tx time
} tag_ctx_S;

// Function prototypes
void tag_init(node_S* n, tag_ctx_S* ctx, const int* anchor_ids, int num_anchors);
extern const node_ops_S tag_ops;
