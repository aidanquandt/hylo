#include "node.h"

void node_init(node_S* n, int id, node_role_E role, vec3_S pos, radio_S* r, tdma_S* tdm, const node_ops_S* ops, void* role_ctx)
{
    n->id = id;
    n->role = role;
    n->pos = pos;
    n->radio = r;
    n->tdma = tdm;
    n->ops = ops;
    n->role_ctx = role_ctx;
}

void node_receive(node_S* n, const message_S* msg)
{
    if (n->ops && n->ops->on_rx) n->ops->on_rx(n, msg);
}

void node_slot_start(node_S* n)
{
    if (n->ops && n->ops->on_slot_start) n->ops->on_slot_start(n);
}

void node_process(node_S* n, uint64_t now_ns)
{
    if (n->ops && n->ops->on_process) n->ops->on_process(n, now_ns);
}
