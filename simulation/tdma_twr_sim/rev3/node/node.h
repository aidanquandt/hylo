#pragma once
#include "common.h"
#include "messages.h"

typedef struct radio_S radio_S;
typedef struct tdma_S tdma_S;

typedef enum
{
    node_tag = 1,
    node_anchor = 2
} node_role_E;

// Forward typedef for tagged struct node_S
typedef struct node_S node_S;

typedef struct
{
    void (*on_rx)(node_S* n, const message_S* msg);      // role-specific
    void (*on_slot_start)(node_S* n);                    // role-specific
    void (*on_process)(node_S* n, uint64_t now_ns);      // optional
} node_ops_S;

// Define the tagged struct node_S
struct node_S
{
    int id;
    node_role_E role;
    vec3_S pos;
    radio_S* radio;
    tdma_S* tdma;
    const node_ops_S* ops;
    void* role_ctx; // tag_ctx_S* or anchor_ctx_S*
};

// Function prototypes
void node_init(node_S* n, int id, node_role_E role, vec3_S pos, radio_S* r, tdma_S* tdm, const node_ops_S* ops, void* role_ctx);
void node_receive(node_S* n, const message_S* msg); // called by radio
void node_slot_start(node_S* n);                    // called by TDMA
void node_process(node_S* n, uint64_t now_ns);      // optional

// Private Functions
static inline vec3_S node_get_pos(const node_S* n)
{
    return n->pos;
}
