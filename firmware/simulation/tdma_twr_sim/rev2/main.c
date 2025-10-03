#include <stdio.h>
#include <string.h>
#include "common.h"
#include "radio.h"
#include "tdma.h"
#include "node.h"
#include "tag.h"
#include "anchor.h"

int main(void) {
    radio_S r;
    tdma_S  tdm;
    radio_init(&r);
    tdma_init(&tdm, &r, s_to_ns(0.050)); // 50 ms slot

    // Build a small network: 3 anchors, 2 tags (only one tag active per slot)
    node_S anchors[3];
    anchor_ctx_S a_ctx[3];
    int anchor_ids[3] = { 101, 102, 103 };
    vec3_S a_pos[3] = { {0,0,2}, {10,0,2}, {0,10,2} };
    for (int i = 0; i < 3; ++i) {
        node_init(&anchors[i], anchor_ids[i], node_anchor, a_pos[i], &r, &tdm, &anchor_ops, &a_ctx[i]);
        anchor_init(&anchors[i], &a_ctx[i], s_to_ns(0.0003)); // 300 us processing
        radio_register(&r, &anchors[i]);
    }

    node_S tags[2];
    tag_ctx_S t_ctx[2];
    int tag_ids[2] = { 201, 202 };
    vec3_S t_pos[2] = { {3,4,1}, {7,2,1} };
    for (int i = 0; i < 2; ++i) {
        node_init(&tags[i], tag_ids[i], node_tag, t_pos[i], &r, &tdm, &tag_ops, &t_ctx[i]);
        tag_init(&tags[i], &t_ctx[i], anchor_ids, arr_len(anchor_ids));
        radio_register(&r, &tags[i]);
    }

    tdma_set_tags(&tdm, tag_ids, arr_len(tag_ids));

    // Simulation loop
    uint64_t sim_end = s_to_ns(1.0); // 1 second
    uint64_t process_ns = s_to_ns(0.001); // 1 ms process cadence for app logic
    for (uint64_t t = 0; t <= sim_end; t += process_ns) {
        tdma_update(&tdm, t);
        // deliver any radio events up to time t
        radio_run_until(&r, t);
        // shared process for all nodes
        for (int i = 0; i < arr_len(tags); ++i) node_process(&tags[i], t);
        for (int i = 0; i < arr_len(anchors); ++i) node_process(&anchors[i], t);
        // Optional: notify slot start explicitly when broadcast arrives.
        // The tag's on_slot_start is driven indirectly by checking tdma_active_tag_id in on_process.
        // To immediately inform nodes at boundaries, you could call node_slot_start on all nodes here.
        if ((t % tdm.slot_dur_ns) == 0) {
            for (int i = 0; i < arr_len(tags); ++i) node_slot_start(&tags[i]);
            for (int i = 0; i < arr_len(anchors); ++i) node_slot_start(&anchors[i]);
        }
    }

    // Drain remaining radio messages (if any)
    radio_run_until(&r, sim_end + s_to_ns(0.1));
    return 0;
}
