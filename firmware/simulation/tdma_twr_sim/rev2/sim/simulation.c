#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <signal.h>
#include "node.h"
#include "simulation.h"
#include "timebase.h"
#include "phy.h"
#include "topology.h"
#include "trace_vcd.h"

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
int current_slot_tag_id = -1;         // TDMA: current active tag slot
static int g_verbose = 0;             // 0=silent (default), 1=steps+summary, 2=+per-message logs

// Graceful stop on Ctrl+C / SIGTERM
static volatile sig_atomic_t g_stop_requested = 0;
static void on_sigint(int sig) { (void)sig; g_stop_requested = 1; }
static void init_signals(void) {
    signal(SIGINT, on_sigint);
#ifdef SIGTERM
    signal(SIGTERM, on_sigint);
#endif
}

// Verbose helpers
#define LOG1(...) do { if (g_verbose >= 1) printf(__VA_ARGS__); } while (0)
#define LOG2(...) do { if (g_verbose >= 2) printf(__VA_ARGS__); } while (0)

static void init_verbosity(void) {
    const char* v = getenv("SIM_VERBOSE");
    if (!v) return;
    int lvl = atoi(v);
    if (lvl >= 0 && lvl <= 3) g_verbose = lvl;
}

// -----------------------------------------------------------------------------
// Local helpers (printing, scheduler)
// -----------------------------------------------------------------------------
static void simulate_round(void) {
    for (int i = 0; i < node_count; i++) {
        nodes[i]->process(nodes[i]);
    }
}

static const char* tag_state_str(int state) {
    switch (state) {
        case TAG_IDLE: return "IDLE";
        case TAG_WAIT_FOR_SLOT: return "WAIT_FOR_SLOT";
        case TAG_INITIATE_TWR: return "INITIATE_TWR";
        case TAG_WAIT_FOR_TWR_RESPONSES: return "WAIT_FOR_TWR_RESPONSES";
        case TAG_END_SLOT: return "END_SLOT";
        default: return "UNKNOWN";
    }
}
static const char* anchor_state_str(int state) {
    switch (state) {
        case ANCHOR_IDLE: return "IDLE";
        default: return "UNKNOWN";
    }
}
static const char* role_str_monitor(Role r) { return r == ANCHOR ? "Anchor" : "Tag"; }
static const char* msg_type_str(MsgType t) {
    switch (t) {
        case MSG_ANNOUNCE: return "ANNOUNCE";
        case MSG_BEACON:   return "BEACON";
        case MSG_TWR_REQUEST: return "TWR_REQUEST";
        case MSG_TWR_RESPONSE: return "TWR_RESPONSE";
        default: return "UNKNOWN";
    }
}

static void print_neighbor_tables(void) {
    LOG1("Simulation: Neighbor tables:\n");
    for (int n = 0; n < node_count; n++) {
        LOG1("  Node %3d (%-6s):", nodes[n]->id, nodes[n]->role == ANCHOR ? "Anchor" : "Tag");
        NeighborTable *table = &neighbor_tables[nodes[n]->id];
        for (int j = 0; j < table->neighbor_count; j++) LOG1(" %d", table->neighbors[j]);
        LOG1("\n");
    }
}
static void print_node_states(const char* header) {
    LOG1("Simulation: Node states %s:\n", header);
    for (int n = 0; n < node_count; n++) {
        LOG1("  Node %3d (%-6s): %s\n",
             nodes[n]->id,
             nodes[n]->role == ANCHOR ? "Anchor" : "Tag",
             nodes[n]->role == TAG ? tag_state_str(nodes[n]->state)
                                   : anchor_state_str(nodes[n]->state));
    }
}

// Hooks called by node.c (SEND/RECV without states)
void on_message_sent(const Message* msg) {
    const char* from_role = (msg->from_id >= 0) ? role_str_monitor(nodes[msg->from_id]->role) : "System";
    const char* to_role   = (msg->to_id   >= 0) ? role_str_monitor(nodes[msg->to_id]->role)   : "System";
    LOG2("[SEND] [%-6s %3d] -> [%-6s %3d]: %s\n",
         from_role, msg->from_id,
         to_role,   msg->to_id,
         msg_type_str(msg->type));
}
void on_message_received(const Message* msg, int receiver_id) {
    const char* from_role = (msg->from_id >= 0) ? role_str_monitor(nodes[msg->from_id]->role) : "System";
    const char* to_role   = (receiver_id >= 0) ? role_str_monitor(nodes[receiver_id]->role)   : "System";
    LOG2("[RECV] [%-6s %3d] -> [%-6s %3d]: %s\n",
         from_role, msg->from_id,
         to_role,   receiver_id,
         msg_type_str(msg->type));
}

// -----------------------------------------------------------------------------
// Initialization (nodes + neighbor discovery from grid)
// -----------------------------------------------------------------------------
static void init_nodes(void) {
    // Create nodes (adjust as needed)
    add_node(0, ANCHOR, process_anchor);
    add_node(1, ANCHOR, process_anchor);
    add_node(2, TAG,    process_tag);
    add_node(3, TAG,    process_tag);

    const int DIST_THRESH = 20; // Manhattan distance threshold (in grid cells)

    // Clear existing neighbor tables
    for (int id = 0; id < MAX_NODES; id++) {
        neighbor_tables[id].neighbor_count = 0;
    }

    // Track positions by node id; initialize to -1 (not placed)
    int posX[MAX_NODES]; int posY[MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) { posX[i] = -1; posY[i] = -1; }

    // Parse the global grid and assign positions to existing nodes
    for (int y = 0; y < PHY_GRID_H; y++) {
        for (int x = 0; x < PHY_GRID_W; x++) {
            int v = PHY_GRID[y][x];
            if (v > 0) {
                int id = v - 1; // grid value maps to node id
                int exists = 0;
                for (int i = 0; i < node_count; i++) {
                    if (nodes[i]->id == id) { exists = 1; break; }
                }
                if (exists) { posX[id] = x; posY[id] = y; }
            }
        }
    }

    // Neighbor discovery via ANNOUNCE (simulate search instead of manually adding)
    // For every pair of nodes within the threshold, enqueue ANNOUNCE to each other.
    for (int a = 0; a < node_count; a++) {
        int idA = nodes[a]->id;
        if (posX[idA] < 0) continue; // not placed in grid
        for (int b = a + 1; b < node_count; b++) {
            int idB = nodes[b]->id;
            if (posX[idB] < 0) continue; // not placed
            int dx = posX[idA] - posX[idB]; if (dx < 0) dx = -dx;
            int dy = posY[idA] - posY[idB]; if (dy < 0) dy = -dy;
            int manhattan = dx + dy;
            if (manhattan <= DIST_THRESH) {
                // A -> B
                Message m1 = (Message){ MSG_ANNOUNCE, idA, idB, 0 };
                on_message_sent(&m1);
                (void)enqueue_msg(&nodes[idB]->next_queue, &m1);
                // B -> A
                Message m2 = (Message){ MSG_ANNOUNCE, idB, idA, 0 };
                on_message_sent(&m2);
                (void)enqueue_msg(&nodes[idA]->next_queue, &m2);
            }
        }
    }

    // Deliver and process the ANNOUNCEs so neighbor_tables are populated via update_neighbor()
    LOG1("Simulation: Neighbor discovery via ANNOUNCE...\n");
    phy_advance_message_queues(); // deliver queued ANNOUNCEs into inboxes
    simulate_round();             // process ANNOUNCEs (nodes update neighbor tables on receive)
}

// Compact end-of-run summary
static void print_summary(void) {
    LOG1("\n========== Summary ==========\n");
    for (int i = 0; i < node_count; i++) {
        Node* n = nodes[i];
        NeighborTable* t = &neighbor_tables[n->id];
        if (n->role == TAG) {
            unsigned long age = n->last_beacon_ms ? (g_sim_time_ms - n->last_beacon_ms) : 0;
            LOG1("Tag    %2d: neighbors=%d, ranges=%d, last_beacon_age_ms=%lu, synced=%d\n",
                 n->id, t->neighbor_count, n->range_count, age, n->synced);
        } else {
            LOG1("Anchor %2d: neighbors=%d\n", n->id, t->neighbor_count);
        }
    }
    LOG1("================================\n");
}

// -----------------------------------------------------------------------------
// Main simulation run loop (per-tick scheduler, 1 ms)
// -----------------------------------------------------------------------------
void simulation_run(void) {
    init_verbosity();
    init_nodes();
    init_signals();

    // Build TDMA tag list
    int tag_ids[MAX_NODES];
    int num_tags = 0;
    for (int i = 0; i < node_count; i++) {
        if (nodes[i]->role == TAG) tag_ids[num_tags++] = nodes[i]->id;
    }

    // TDMA timing (1 Hz frame): control window + equal-duration slots fill 1000 ms
    const int FRAME_PERIOD_MS   = 1000;                 // 1 Hz TDMA frame
    const int CONTROL_PERIOD_MS = 10;                   // control window duration at frame start
    const int SLOT_PERIOD_MS    = (num_tags > 0)
                                  ? (FRAME_PERIOD_MS - CONTROL_PERIOD_MS) / num_tags
                                  : (FRAME_PERIOD_MS - CONTROL_PERIOD_MS);
    const int TOTAL_FRAMES      = 5;                    // run for N frames
    const unsigned long END_TIME_MS = (unsigned long)TOTAL_FRAMES * FRAME_PERIOD_MS;

    const int coordinator_anchor_id = 0;
    const int beacon_ttl = 3;

    // Start VCD trace (open with GTKWave)
    vcd_start("trace.vcd");

    // Tick-based scheduler: 1 ms per tick
    while (g_sim_time_ms < END_TIME_MS) {
        if (g_stop_requested) break;
        unsigned long t_in_frame = g_sim_time_ms % FRAME_PERIOD_MS;

        // Frame start: emit BEACON once per frame at t=0
        if (t_in_frame == 0) {
            current_slot_tag_id = -1;
            LOG1("\n========== TDMA Control (Frame %lu: Beacon) ==========\n", (g_sim_time_ms / FRAME_PERIOD_MS) + 1);
            print_neighbor_tables();
            print_node_states("before control");
            LOG1("Simulation: Messages this control tick (Beacon flood):\n");
            emit_beacon(coordinator_anchor_id, beacon_ttl);
        }

        // Compute current slot owner for this tick
        if (t_in_frame < CONTROL_PERIOD_MS) {
            // Control window: no slot owner
            current_slot_tag_id = -1;
        } else {
            if (num_tags > 0) {
                int elapsed = (int)(t_in_frame - CONTROL_PERIOD_MS);
                int slot_idx = elapsed / SLOT_PERIOD_MS;
                if (slot_idx >= num_tags) slot_idx = num_tags - 1; // clamp last few ms of frame
                current_slot_tag_id = tag_ids[slot_idx];

                // Slot boundary print (optional, once at slot start)
                if ((elapsed % SLOT_PERIOD_MS) == 0) {
                    LOG1("\n---------- TDMA Slot start (Frame %lu, Slot %d/%d -> Tag %d) ----------\n",
                         (g_sim_time_ms / FRAME_PERIOD_MS) + 1, slot_idx + 1, num_tags, current_slot_tag_id);
                    print_node_states("at slot start");
                }
            } else {
                current_slot_tag_id = -1;
            }
        }

        // One tick: run all node state machines once
        simulate_round();

        // Deliver messages generated this tick for reception on the next tick
        phy_advance_message_queues();

        // Dump VCD for this tick
        vcd_dump_step();

        // End-of-control-window print (optional)
        if (t_in_frame == (CONTROL_PERIOD_MS - 1)) {
            print_node_states("after control");
            LOG1("========== End of TDMA Control ==========\n");
        }

        // Optional: print at slot end to make per-slot activity visible
        if (t_in_frame >= CONTROL_PERIOD_MS && num_tags > 0) {
            int elapsed = (int)(t_in_frame - CONTROL_PERIOD_MS);
            if ((elapsed % SLOT_PERIOD_MS) == (SLOT_PERIOD_MS - 1)) {
                LOG1("---------- TDMA Slot end   (Frame %lu) ----------\n",
                     (g_sim_time_ms / FRAME_PERIOD_MS) + 1);
                print_node_states("at slot end");
            }
        }

        // Advance simulated time by 1 ms
        sim_advance_time_ms(SIM_STEP_MS);
    }

    print_summary();

    // Cleanup
    for (int i = 0; i < node_count; i++) {
        free(nodes[i]);
    }
    vcd_stop();
}