#pragma once
#include "common.h"
#include "messages.h"

typedef struct radio_S radio_S;

typedef struct tdma_S
{
    int tag_ids[32];
    int num_tags;
    uint64_t slot_dur_ns;
    uint64_t frame_start_ns;
    int current_slot; // index into tag_ids
    radio_S* radio;
} tdma_S;

void tdma_init(tdma_S* t, radio_S* r, uint64_t slot_dur_ns);
void tdma_set_tags(tdma_S* t, const int* tag_ids, int count);
int  tdma_active_tag_id(tdma_S* t, uint64_t now_ns);
void tdma_update(tdma_S* t, uint64_t now_ns); // emits slot-start broadcast at slot boundaries
