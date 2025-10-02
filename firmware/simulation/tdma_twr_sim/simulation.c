#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "node.h"
#include "simulation.h"

// TDMA: current active tag slot (visible to node.c)
int current_slot_tag_id = -1;

// Forward declare delivery step provided by node.c
void advance_message_queues(void);

// Deliver all next-step messages into current inboxes (simulation pacing)
void advance_message_queues(void) {
    for (int i = 0; i < node_count; i++) {
        Node* n = nodes[i];
        // Overwrite current inbox with what was scheduled for next step
        n->msg_queue = n->next_queue;
        // Clear next-step queue
        n->next_queue.head = 0;
        n->next_queue.tail = 0;
    }
}

// Initialize neighbors for demo scenario using a 2D int grid geometry.
// 0 = empty; v > 0 places node with id = v - 1 at that cell.
// Nodes within DIST_THRESH (Manhattan distance) are considered neighbors.
void init_neighbors() {
    // Editable int grid (all rows must have equal length)
    // Example maps: 1->node id 0, 2->node id 1, 3->node id 2, 4->node id 3
    static const int GRID_H = 10;
    static const int GRID_W = 10;
    static const int GRID[10][10] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 2},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 3, 0, 0, 0, 4, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };
    const int DIST_THRESH = 20; // Manhattan distance threshold (in grid cells)

    // Clear existing neighbor tables
    for (int id = 0; id < MAX_NODES; id++) {
        neighbor_tables[id].neighbor_count = 0;
    }

    // Track positions by node id; initialize to -1 (not placed)
    int posX[MAX_NODES]; int posY[MAX_NODES];
    for (int i = 0; i < MAX_NODES; i++) { posX[i] = -1; posY[i] = -1; }

    // Parse the grid and assign positions to existing nodes
    for (int y = 0; y < GRID_H; y++) {
        for (int x = 0; x < GRID_W; x++) {
            int v = GRID[y][x];
            if (v > 0) {
                int id = v - 1; // grid value maps to node id
                // Only assign if such a node exists in this simulation
                int exists = 0;
                for (int i = 0; i < node_count; i++) {
                    if (nodes[i]->id == id) { exists = 1; break; }
                }
                if (exists) { posX[id] = x; posY[id] = y; }
            }
        }
    }

    // Build neighbors based on Manhattan distance threshold
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
                add_neighbor(idA, idB);
                add_neighbor(idB, idA);
            }
        }
    }
}

// Simulation loop that calls node processing functions
void simulate_round() {
    for (int i = 0; i < node_count; i++) {
        nodes[i]->process(nodes[i]);
    }
}

// State-to-string helpers (TDMA)
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
static const char* node_state_str(int id) {
    if (id < 0) return "-";
    return nodes[id]->role == TAG ? tag_state_str(nodes[id]->state)
                                  : anchor_state_str(nodes[id]->state);
}

// Hooks called by node.c (SEND/RECV without states)
void on_message_sent(const Message* msg) {
    const char* from_role = (msg->from_id >= 0) ? role_str_monitor(nodes[msg->from_id]->role) : "System";
    const char* to_role   = (msg->to_id   >= 0) ? role_str_monitor(nodes[msg->to_id]->role)   : "System";
    printf("[SEND] [%-6s %3d] -> [%-6s %3d]: %s\n",
        from_role, msg->from_id,
        to_role,   msg->to_id,
        msg_type_str(msg->type));
}
void on_message_received(const Message* msg, int receiver_id) {
    const char* from_role = (msg->from_id >= 0) ? role_str_monitor(nodes[msg->from_id]->role) : "System";
    const char* to_role   = (receiver_id >= 0) ? role_str_monitor(nodes[receiver_id]->role)   : "System";
    printf("[RECV] [%-6s %3d] -> [%-6s %3d]: %s\n",
        from_role, msg->from_id,
        to_role,   receiver_id,
        msg_type_str(msg->type));
}

// Intra-step draining: allow fast events (e.g., TWR responses) to be received in the same TDMA step
#define MAX_INTRA_STEP_PASSES 4

static int has_pending_next(void) {
    for (int i = 0; i < node_count; i++) {
        if (nodes[i]->next_queue.head != nodes[i]->next_queue.tail) return 1;
    }
    return 0;
}

static void run_intra_step(void) {
    for (int pass = 0; pass < MAX_INTRA_STEP_PASSES; pass++) {
        simulate_round();
        if (!has_pending_next()) break;
        // Deliver messages generated in this pass so they can be handled within the same step
        advance_message_queues();
    }
}

void simulation_run(void) {
    add_node(0, ANCHOR, process_anchor);
    add_node(1, ANCHOR, process_anchor);
    add_node(2, TAG,    process_tag);
    add_node(3, TAG,    process_tag);

    init_neighbors();

    // Build TDMA tag list
    int tag_ids[MAX_NODES];
    int num_tags = 0;
    for (int i = 0; i < node_count; i++) {
        if (nodes[i]->role == TAG) tag_ids[num_tags++] = nodes[i]->id;
    }

    int slot_idx = 0;    // 0..num_tags-1
    int frame_idx = 0;   // increments when slot_idx wraps
    const int coordinator_anchor_id = 0;
    const int beacon_ttl = 3;

    for (int step = 0; step < 10; step++) {
        // Control step at frame start: emit and process beacon only
        if (slot_idx == 0 && nodes[coordinator_anchor_id] && nodes[coordinator_anchor_id]->role == ANCHOR) {
            current_slot_tag_id = -1;
            printf("\n========== TDMA Control (Frame %d: Beacon) ==========\n", frame_idx + 1);

            // Neighbor tables before control step
            printf("Simulation: Neighbor tables before step:\n");
            for (int n = 0; n < node_count; n++) {
                printf("  Node %3d (%-6s):", nodes[n]->id, nodes[n]->role == ANCHOR ? "Anchor" : "Tag");
                NeighborTable *table = &neighbor_tables[nodes[n]->id];
                for (int j = 0; j < table->neighbor_count; j++) printf(" %d", table->neighbors[j]);
                printf("\n");
            }

            // Node states before control step
            printf("Simulation: Node states before step:\n");
            for (int n = 0; n < node_count; n++) {
                printf("  Node %3d (%-6s): %s\n",
                       nodes[n]->id,
                       nodes[n]->role == ANCHOR ? "Anchor" : "Tag",
                       nodes[n]->role == TAG ? tag_state_str(nodes[n]->state)
                                             : anchor_state_str(nodes[n]->state));
            }

            printf("Simulation: Messages this step (Beacon flood):\n");
            emit_beacon(coordinator_anchor_id, beacon_ttl);
            run_intra_step();
            // Deliver control-plane messages to be received in slot 1
            advance_message_queues();

            // Node states after control step
            printf("Simulation: Node states after step:\n");
            for (int n = 0; n < node_count; n++) {
                printf("  Node %3d (%-6s): %s\n",
                       nodes[n]->id,
                       nodes[n]->role == ANCHOR ? "Anchor" : "Tag",
                       nodes[n]->role == TAG ? tag_state_str(nodes[n]->state)
                                             : anchor_state_str(nodes[n]->state));
            }

            printf("========== End of TDMA Control ==========\n");
        }

        // Assign current slot owner (visible to node.c via extern)
        current_slot_tag_id = (num_tags > 0) ? tag_ids[slot_idx] : -1;

        printf("\n========== TDMA Step %d (Frame %d, Slot %d/%d -> Tag %d) ==========\n",
               step + 1, frame_idx + 1, slot_idx + 1, num_tags, current_slot_tag_id);

        // Neighbor tables before slot step
        printf("Simulation: Neighbor tables before step:\n");
        for (int n = 0; n < node_count; n++) {
            printf("  Node %3d (%-6s):", nodes[n]->id, nodes[n]->role == ANCHOR ? "Anchor" : "Tag");
            NeighborTable *table = &neighbor_tables[nodes[n]->id];
            for (int j = 0; j < table->neighbor_count; j++) printf(" %d", table->neighbors[j]);
            printf("\n");
        }

        // Node states before slot step
        printf("Simulation: Node states before step:\n");
        for (int n = 0; n < node_count; n++) {
            printf("  Node %3d (%-6s): %s\n",
                   nodes[n]->id,
                   nodes[n]->role == ANCHOR ? "Anchor" : "Tag",
                   nodes[n]->role == TAG ? tag_state_str(nodes[n]->state)
                                         : anchor_state_str(nodes[n]->state));
        }

        // Messages header
        printf("Simulation: Messages this step:\n");
        run_intra_step();

        // Deliver messages to be received in the next TDMA step
        advance_message_queues();

        // Node states after slot step
        printf("Simulation: Node states after step:\n");
        for (int n = 0; n < node_count; n++) {
            printf("  Node %3d (%-6s): %s\n",
                   nodes[n]->id,
                   nodes[n]->role == ANCHOR ? "Anchor" : "Tag",
                   nodes[n]->role == TAG ? tag_state_str(nodes[n]->state)
                                         : anchor_state_str(nodes[n]->state));
        }

        printf("========== End of TDMA Step %d ==========\n", step + 1);

        // Advance to next tag slot; wrap increments frame index
        if (num_tags > 0) {
            slot_idx = (slot_idx + 1) % num_tags;
            if (slot_idx == 0) frame_idx++;
        }
    }

    // Cleanup
    for (int i = 0; i < node_count; i++) {
        free(nodes[i]);
    }
}