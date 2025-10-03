#include <stdio.h>
#include <string.h>
#include "node.h"
#include "tag.h"
#include "radio.h"
#include "tdma.h"

static void tag_send_poll(node_S* n, tag_ctx_S* ctx, int anchor_id) {
    radio_S* r = n->radio;
    uint64_t now = radio_now(r);
    ctx->t1_ns = now;
    message_S m = {0};
    m.type = msg_twr_poll;
    m.src_id = n->id;
    m.dst_id = anchor_id;
    m.tx_time_ns = now;
    m.u.poll.t1_ns = ctx->t1_ns;
    radio_send(r, &m);
    ctx->st = TAG_WAIT_RESP;
}

static void tag_on_rx(node_S* n, const message_S* msg) {
    tag_ctx_S* ctx = (tag_ctx_S*)n->role_ctx;
    if (msg->type == msg_tdma_slot_start) {
        // ignore here; on_slot_start will manage
        return;
    }
    if (msg->type == msg_twr_resp && ctx->st == TAG_WAIT_RESP) {
        uint64_t t4 = msg->rx_time_ns;
        uint64_t t1 = ctx->t1_ns;
        uint64_t t2 = msg->u.resp.t2_ns;
        uint64_t t3 = msg->u.resp.t3_ns;
        // Double-sided TWR ToF
        // round1 = t4 - t1; reply = t3 - t2; ToF = (round1 - reply)/2
        double tof_s = ((double)(t4 - t1) - (double)(t3 - t2)) * 1e-9 / 2.0;
        double dist_m = tof_s * c_mps;
        printf("[TAG %d] Range to ANCHOR %d = %.3f m (t1=%llu t2=%llu t3=%llu t4=%llu)\n",
               n->id, msg->src_id, dist_m,
               (unsigned long long)t1, (unsigned long long)t2,
               (unsigned long long)t3, (unsigned long long)t4);
        ctx->st = TAG_IDLE;
        // Next anchor if time remains; we drive this via on_process to avoid immediate chain
    }
}

static void tag_on_slot_start(node_S* n) {
    tag_ctx_S* ctx = (tag_ctx_S*)n->role_ctx;
    // Only act if this tag owns the slot
    int active = tdma_active_tag_id(n->tdma, radio_now(n->radio));
    if (active != n->id) return;
    ctx->idx = 0;
    ctx->st = TAG_IDLE;
}

static void tag_on_process(node_S* n, uint64_t now_ns) {
    tag_ctx_S* ctx = (tag_ctx_S*)n->role_ctx;
    int active = tdma_active_tag_id(n->tdma, now_ns);
    if (active != n->id) return;
    if (ctx->st == TAG_IDLE && ctx->idx < ctx->num_anchors) {
        int anchor_id = ctx->anchors[ctx->idx++];
        tag_send_poll(n, ctx, anchor_id);
    }
    // If WAIT_RESP, do nothing; response handler will flip to IDLE and next process continues.
}

void tag_init(node_S* n, tag_ctx_S* ctx, const int* anchor_ids, int num_anchors) {
    memset(ctx, 0, sizeof(*ctx));
    if (num_anchors > arr_len(ctx->anchors)) num_anchors = arr_len(ctx->anchors);
    memcpy(ctx->anchors, anchor_ids, num_anchors * sizeof(int));
    ctx->num_anchors = num_anchors;
    n->role_ctx = ctx;
}

const node_ops_S tag_ops = {
    .on_rx = tag_on_rx,
    .on_slot_start = tag_on_slot_start,
    .on_process = tag_on_process,
};
