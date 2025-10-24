#include <string.h>
#include <stdio.h>
#include "radio.h"
#include "node.h"

// Private Functions
static uint64_t compute_propagation_ns(node_S* a, node_S* b) {
    vec3_S pa = node_get_pos(a), pb = node_get_pos(b);
    double d = vec3_dist(&pa, &pb); // meters
    double tof_s = d / c_mps;
    return s_to_ns(tof_s);
}

// Public functions
void radio_init(radio_S* r) {
    memset(r, 0, sizeof(*r));
    r->now_ns = 0;
}

void radio_register(radio_S* r, node_S* n) {
    if (r->node_count < arr_len(r->nodes)) {
        r->nodes[r->node_count++] = n;
    }
}

node_S* radio_find_node(radio_S* r, int id) {
    for (int i = 0; i < r->node_count; ++i) if (r->nodes[i]->id == id) return r->nodes[i];
    return NULL;
}

uint64_t radio_now(radio_S* r) { return r->now_ns; }

void radio_send(radio_S* r, const message_S* m) {
    // Broadcast if dst_id < 0
    if (m->dst_id < 0) {
        for (int i = 0; i < r->node_count; ++i) {
            node_S* dst = r->nodes[i];
            if (dst->id == m->src_id) continue;
            if (r->q_count >= arr_len(r->queue)) continue;
            delivery_S* d = &r->queue[r->q_count++];
            d->msg = *m;
            d->msg.rx_time_ns = 0; // filled on delivery
            node_S* src = radio_find_node(r, m->src_id);
            uint64_t prop = src ? compute_propagation_ns(src, dst) : 0;
            d->arrival_ns = m->tx_time_ns + prop;
            // Overwrite dst for broadcast copy
            d->msg.dst_id = dst->id;
        }
        return;
    }
    node_S* dst = radio_find_node(r, m->dst_id);
    if (!dst || r->q_count >= arr_len(r->queue)) return;
    delivery_S* d = &r->queue[r->q_count++];
    d->msg = *m;
    node_S* src = radio_find_node(r, m->src_id);
    uint64_t prop = src ? compute_propagation_ns(src, dst) : 0;
    d->arrival_ns = m->tx_time_ns + prop;
}

void radio_run_until(radio_S* r, uint64_t until_ns) {
    // advance time and deliver any due messages in time order (simple linear scan)
    while (1) {
        // find earliest due
        int idx = -1;
        uint64_t best = UINT64_MAX;
        for (int i = 0; i < r->q_count; ++i) {
            if (r->queue[i].arrival_ns < best) { best = r->queue[i].arrival_ns; idx = i; }
        }
        if (idx < 0 || best > until_ns) break;
        // deliver
        r->now_ns = best;
        delivery_S d = r->queue[idx];
        // compact queue
        memmove(&r->queue[idx], &r->queue[idx+1], (r->q_count - idx - 1) * sizeof(delivery_S));
        r->q_count--;
        node_S* dst = radio_find_node(r, d.msg.dst_id);
        if (!dst) continue;
        message_S m = d.msg;
        m.rx_time_ns = r->now_ns;
        node_receive(dst, &m);
    }
    // advance time to until_ns
    r->now_ns = until_ns;
}
