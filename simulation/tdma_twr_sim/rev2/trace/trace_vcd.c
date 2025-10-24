#include <stdio.h>
#include <string.h>
#include "node.h"
#include "timebase.h"

static FILE* vcd = NULL;

static unsigned char tx_pulse[MAX_NODES] = {0};
static unsigned char rx_pulse[MAX_NODES] = {0};
static unsigned char last_tx_type[MAX_NODES] = {0};
static unsigned char last_rx_type[MAX_NODES] = {0};

// Max printable widths (characters) for ASCII signals
#define STATE_STR_MAXCH 16
#define TYPE_STR_MAXCH  12

// Enum -> string helpers (local copies for VCD text)
static const char* tag_state_name(int s) {
    switch (s) {
        case TAG_IDLE: return "IDLE";
        case TAG_WAIT_FOR_SLOT: return "WAIT_FOR_SLOT";
        case TAG_INITIATE_TWR: return "INITIATE_TWR";
        case TAG_WAIT_FOR_TWR_RESPONSES: return "WAIT_TWR_RESP";
        case TAG_END_SLOT: return "END_SLOT";
        default: return "UNKNOWN";
    }
}
static const char* anchor_state_name(int s) {
    switch (s) {
        case ANCHOR_IDLE: return "IDLE";
        default: return "UNKNOWN";
    }
}
static const char* msg_type_name(unsigned char t) {
    switch (t) {
        case MSG_ANNOUNCE: return "ANNOUNCE";
        case MSG_BEACON: return "BEACON";
        case MSG_TWR_REQUEST: return "TWR_REQUEST";
        case MSG_TWR_RESPONSE: return "TWR_RESPONSE";
        default: return "";
    }
}

// Write an ASCII string as a fixed-width (chars*8) VCD vector (left-to-right, space-padded)
static void vcd_write_ascii_fixed(const char* s, int width_chars, const char* id) {
    int len = (int)strlen(s);
    if (len > width_chars) len = width_chars;
    fputc('b', vcd);
    // Emit each char as 8 bits (MSB first)
    for (int i = 0; i < width_chars; ++i) {
        unsigned char c = (i < len) ? (unsigned char)s[i] : (unsigned char)' ';
        for (int b = 7; b >= 0; --b) {
            fputc(((c >> b) & 1) ? '1' : '0', vcd);
        }
    }
    fprintf(vcd, " %s\n", id);
}

static void vcd_write_bin(unsigned int val, int width, const char* id) {
    char buf[64];
    if (width > (int)(sizeof(buf) - 2)) width = (int)sizeof(buf) - 2;
    for (int i = width - 1; i >= 0; --i) buf[width - 1 - i] = ((val >> i) & 1) ? '1' : '0';
    buf[width] = '\0';
    fprintf(vcd, "b%s %s\n", buf, id);
}

void vcd_start(const char* path) {
    vcd = fopen(path, "w");
    if (!vcd) return;

    fprintf(vcd, "$date\n  -\n$end\n");
    fprintf(vcd, "$version\n  tdma_twr_sim\n$end\n");
    fprintf(vcd, "$timescale %d ms $end\n", SIM_STEP_MS);
    fprintf(vcd, "$scope module sim $end\n");
    fprintf(vcd, "$var wire 8 cs current_slot $end\n");

    for (int i = 0; i < node_count; i++) {
        int id = nodes[i]->id;
        fprintf(vcd, "$var wire 8 s%d node%d_state $end\n", id, id);
        fprintf(vcd, "$var wire 1 y%d node%d_synced $end\n", id, id);
        fprintf(vcd, "$var wire 8 n%d node%d_neighbors $end\n", id, id);
        fprintf(vcd, "$var wire 8 r%d node%d_range_count $end\n", id, id);
        fprintf(vcd, "$var wire 1 tx%d node%d_tx $end\n", id, id);
        fprintf(vcd, "$var wire 3 tt%d node%d_tx_type $end\n", id, id);
        fprintf(vcd, "$var wire 1 rx%d node%d_rx $end\n", id, id);
        fprintf(vcd, "$var wire 3 rt%d node%d_rx_type $end\n", id, id);
        fprintf(vcd, "$var wire 8 qi%d node%d_inbox_depth $end\n", id, id);
        fprintf(vcd, "$var wire 8 qn%d node%d_next_depth $end\n", id, id);

        // New ASCII mirrors for readability in GTKWave (set Data Format -> ASCII)
        fprintf(vcd, "$var wire %d ss%d node%d_state_txt $end\n", STATE_STR_MAXCH * 8, id, id);
        fprintf(vcd, "$var wire %d tts%d node%d_tx_type_txt $end\n", TYPE_STR_MAXCH * 8, id, id);
        fprintf(vcd, "$var wire %d rts%d node%d_rx_type_txt $end\n", TYPE_STR_MAXCH * 8, id, id);
    }

    fprintf(vcd, "$upscope $end\n$enddefinitions $end\n");
    fprintf(vcd, "$dumpvars\n");
    vcd_write_bin((unsigned int)current_slot_tag_id, 8, "cs");
    for (int i = 0; i < node_count; i++) {
        int id = nodes[i]->id;
        char sid[8], yid[8], nid[8], rid[8], txid[8], ttid[8], rxid[8], rtid[8], qii[8], qni[8];
        char ssid[8], ttsid[8], rtsid[8];
        snprintf(sid, sizeof(sid), "s%d", id);
        snprintf(yid, sizeof(yid), "y%d", id);
        snprintf(nid, sizeof(nid), "n%d", id);
        snprintf(rid, sizeof(rid), "r%d", id);
        snprintf(txid, sizeof(txid), "tx%d", id);
        snprintf(ttid, sizeof(ttid), "tt%d", id);
        snprintf(rxid, sizeof(rxid), "rx%d", id);
        snprintf(rtid, sizeof(rtid), "rt%d", id);
        snprintf(qii, sizeof(qii), "qi%d", id);
        snprintf(qni, sizeof(qni), "qn%d", id);
        snprintf(ssid, sizeof(ssid), "ss%d", id);
        snprintf(ttsid, sizeof(ttsid), "tts%d", id);
        snprintf(rtsid, sizeof(rtsid), "rts%d", id);

        vcd_write_bin((unsigned int)nodes[i]->state, 8, sid);
        fprintf(vcd, "%c%s\n", nodes[i]->synced ? '1' : '0', yid);
        vcd_write_bin((unsigned int)neighbor_tables[id].neighbor_count, 8, nid);
        vcd_write_bin((unsigned int)nodes[i]->range_count, 8, rid);
        fprintf(vcd, "0%s\n", txid);
        vcd_write_bin(0, 3, ttid);
        fprintf(vcd, "0%s\n", rxid);
        vcd_write_bin(0, 3, rtid);
        unsigned int inbox = (nodes[i]->msg_queue.tail - nodes[i]->msg_queue.head + MAX_MESSAGES) % MAX_MESSAGES;
        unsigned int nextd = (nodes[i]->next_queue.tail - nodes[i]->next_queue.head + MAX_MESSAGES) % MAX_MESSAGES;
        vcd_write_bin(inbox, 8, qii);
        vcd_write_bin(nextd, 8, qni);

        // Initial ASCII values
        const char* sname = (nodes[i]->role == TAG)
            ? tag_state_name(nodes[i]->state)
            : anchor_state_name(nodes[i]->state);
        vcd_write_ascii_fixed(sname, STATE_STR_MAXCH, ssid);
        vcd_write_ascii_fixed("", TYPE_STR_MAXCH, ttsid);
        vcd_write_ascii_fixed("", TYPE_STR_MAXCH, rtsid);
    }
    fprintf(vcd, "$end\n");
}

void vcd_dump_step(void) {
    if (!vcd) return;
    fprintf(vcd, "#%lu\n", g_sim_time_ms);
    vcd_write_bin((unsigned int)current_slot_tag_id, 8, "cs");
    for (int i = 0; i < node_count; i++) {
        int id = nodes[i]->id;
        char sid[8], yid[8], nid[8], rid[8], txid[8], ttid[8], rxid[8], rtid[8], qii[8], qni[8];
        char ssid[8], ttsid[8], rtsid[8];
        snprintf(sid, sizeof(sid), "s%d", id);
        snprintf(yid, sizeof(yid), "y%d", id);
        snprintf(nid, sizeof(nid), "n%d", id);
        snprintf(rid, sizeof(rid), "r%d", id);
        snprintf(txid, sizeof(txid), "tx%d", id);
        snprintf(ttid, sizeof(ttid), "tt%d", id);
        snprintf(rxid, sizeof(rxid), "rx%d", id);
        snprintf(rtid, sizeof(rtid), "rt%d", id);
        snprintf(qii, sizeof(qii), "qi%d", id);
        snprintf(qni, sizeof(qni), "qn%d", id);
        snprintf(ssid, sizeof(ssid), "ss%d", id);
        snprintf(ttsid, sizeof(ttsid), "tts%d", id);
        snprintf(rtsid, sizeof(rtsid), "rts%d", id);

        vcd_write_bin((unsigned int)nodes[i]->state, 8, sid);
        fprintf(vcd, "%c%s\n", nodes[i]->synced ? '1' : '0', yid);
        vcd_write_bin((unsigned int)neighbor_tables[id].neighbor_count, 8, nid);
        vcd_write_bin((unsigned int)nodes[i]->range_count, 8, rid);

        fprintf(vcd, "%c%s\n", tx_pulse[id] ? '1' : '0', txid);
        vcd_write_bin((unsigned int)last_tx_type[id], 3, ttid);
        fprintf(vcd, "%c%s\n", rx_pulse[id] ? '1' : '0', rxid);
        vcd_write_bin((unsigned int)last_rx_type[id], 3, rtid);
        tx_pulse[id] = 0;
        rx_pulse[id] = 0;

        unsigned int inbox = (nodes[i]->msg_queue.tail - nodes[i]->msg_queue.head + MAX_MESSAGES) % MAX_MESSAGES;
        unsigned int nextd = (nodes[i]->next_queue.tail - nodes[i]->next_queue.head + MAX_MESSAGES) % MAX_MESSAGES;
        vcd_write_bin(inbox, 8, qii);
        vcd_write_bin(nextd, 8, qni);

        // ASCII mirrors
        const char* sname = (nodes[i]->role == TAG)
            ? tag_state_name(nodes[i]->state)
            : anchor_state_name(nodes[i]->state);
        vcd_write_ascii_fixed(sname, STATE_STR_MAXCH, ssid);
        vcd_write_ascii_fixed(msg_type_name(last_tx_type[id]), TYPE_STR_MAXCH, ttsid);
        vcd_write_ascii_fixed(msg_type_name(last_rx_type[id]), TYPE_STR_MAXCH, rtsid);
    }
}

void vcd_stop(void) {
    if (vcd) { fclose(vcd); vcd = NULL; }
}

void trace_phy_tx(const Message* msg) {
    if (!msg || msg->from_id < 0 || msg->from_id >= MAX_NODES) return;
    tx_pulse[msg->from_id] = 1;
    last_tx_type[msg->from_id] = (unsigned char)msg->type;
}

void trace_phy_rx(const Message* msg, int receiver_id) {
    if (!msg || receiver_id < 0 || receiver_id >= MAX_NODES) return;
    rx_pulse[receiver_id] = 1;
    last_rx_type[receiver_id] = (unsigned char)msg->type;
}
