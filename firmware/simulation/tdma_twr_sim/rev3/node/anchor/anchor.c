#include <string.h>
#include "anchor.h"
#include "radio.h"

static void anchor_on_rx(node_S* n, const message_S* msg) {
    anchor_ctx_S* ctx = (anchor_ctx_S*)n->role_ctx;
    if (msg->type == msg_twr_poll) {
        // t2: poll RX time is this message's rx_time_ns
        uint64_t t2 = msg->rx_time_ns;
        // t3: scheduled RESP tx time
        uint64_t t3 = t2 + ctx->resp_proc_delay_ns;
        message_S r = (message_S){0};
        r.type = msg_twr_resp;
        r.src_id = n->id;
        r.dst_id = msg->src_id;
        r.tx_time_ns = t3;
        r.u.resp.t2_ns = t2;
        r.u.resp.t3_ns = t3;
        radio_send(n->radio, &r);
    }
}

static void anchor_on_slot_start(node_S* n) {
    (void)n; // anchors are passive
}

static void anchor_on_process(node_S* n, uint64_t now_ns) {
    (void)n; (void)now_ns;
}

void anchor_init(node_S* n, anchor_ctx_S* ctx, uint64_t resp_proc_delay_ns) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->resp_proc_delay_ns = resp_proc_delay_ns;
    n->role_ctx = ctx;
}

const node_ops_S anchor_ops = {
    .on_rx = anchor_on_rx,
    .on_slot_start = anchor_on_slot_start,
    .on_process = anchor_on_process,
};
