#include "node.h"
#include "phy.h"
#include "trace_vcd.h"

void phy_unicast(const Message* msg) {
    on_message_sent(msg);
    trace_phy_tx(msg);
    if (msg->to_id >= 0 && msg->to_id < MAX_NODES && nodes[msg->to_id]) {
        Message m = *msg;
        (void)enqueue_msg(&nodes[msg->to_id]->next_queue, &m);
    }
}

void phy_broadcast(int from_id, MsgType type, int ttl) {
    if (from_id < 0 || from_id >= MAX_NODES) return;
    NeighborTable* t = &neighbor_tables[from_id];
    for (int i = 0; i < t->neighbor_count; i++) {
        int nid = t->neighbors[i];
        if (nid < 0 || nid >= MAX_NODES || !nodes[nid]) continue;
        Message m = (Message){ type, from_id, nid, ttl };
        on_message_sent(&m);
        trace_phy_tx(&m);
        (void)enqueue_msg(&nodes[nid]->next_queue, &m);
    }
}

void phy_advance_message_queues(void) {
    for (int i = 0; i < node_count; i++) {
        Node* n = nodes[i];
        n->msg_queue = n->next_queue;
        n->next_queue.head = 0;
        n->next_queue.tail = 0;
    }
}
