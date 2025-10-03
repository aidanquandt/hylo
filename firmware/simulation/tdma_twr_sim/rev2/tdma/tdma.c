#include <string.h>
#include "tdma.h"
#include "radio.h"
#include "node.h"

void tdma_init(tdma_S* t, radio_S* r, uint64_t slot_dur_ns) {
    memset(t, 0, sizeof(*t));
    t->radio = r;
    t->slot_dur_ns = slot_dur_ns;
    t->frame_start_ns = 0;
    t->current_slot = -1;
}

void tdma_set_tags(tdma_S* t, const int* tag_ids, int count) {
    if (count > arr_len(t->tag_ids)) count = arr_len(t->tag_ids);
    memcpy(t->tag_ids, tag_ids, count * sizeof(int));
    t->num_tags = count;
}

int tdma_active_tag_id(tdma_S* t, uint64_t now_ns) {
    if (t->num_tags == 0) return -1;
    uint64_t elapsed = now_ns - t->frame_start_ns;
    int slot = (int)(elapsed / t->slot_dur_ns) % t->num_tags;
    return t->tag_ids[slot];
}

void tdma_update(tdma_S* t, uint64_t now_ns) {
    if (t->num_tags == 0) return;
    uint64_t elapsed = now_ns - t->frame_start_ns;
    int slot = (int)(elapsed / t->slot_dur_ns) % t->num_tags;
    if (slot != t->current_slot) {
        t->current_slot = slot;
        int active_tag = t->tag_ids[slot];
        message_S m = {0};
        m.type = msg_tdma_slot_start;
        m.src_id = -999; // TDMA controller
        m.dst_id = -1;   // broadcast
        m.tx_time_ns = now_ns;
        m.u.slot.active_tag_id = active_tag;
        radio_send(t->radio, &m);
    }
}