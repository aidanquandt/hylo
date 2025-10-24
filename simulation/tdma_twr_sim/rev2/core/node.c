#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "node.h"
#include "timebase.h"
#include "phy.h"

// Max neighbors and neighbor timeout in ms
#define NEIGHBOR_TIMEOUT_MS 30000

// TDMA sync validity window (e.g., ~1.5s in sim-time)
#define SYNC_TIMEOUT_MS 10000

// Small processing/relay delays to simulate realistic behavior (ms)
#define ANCHOR_TWR_PROC_DELAY_MS 2
#define BEACON_RELAY_DELAY_MS    1

// Placeholder: Implement this function to return current system time in ms
unsigned long get_current_time_ms() {
    // Return global sim time (advanced explicitly by simulation.c)
    return g_sim_time_ms;
}

// These should only be defined here, not in simulation.c
Node *nodes[MAX_NODES];
int node_count = 0;

// Define neighbor_tables here
NeighborTable neighbor_tables[MAX_NODES];

// Optional: TWR response timeout in ms (sim-time via get_current_time_ms)
#define TWR_RESPONSE_TIMEOUT_MS 500  // adjust as needed

// --- Simple geometry model to make TWR realistic (units: meters) ---
static float pos_x[MAX_NODES] = {0};
static float pos_y[MAX_NODES] = {0};
static int pos_init = 0;

static void ensure_positions(int id, Role role) {
    // Silence unused parameter warnings (positions are static in this sim)
    (void)id; (void)role;
    if (pos_init) return;
    // Example layout for first few nodes; others default to (0,0).
    // Anchor 0: (0,0), Anchor 1: (5,0), Tag 2: (2,2), Tag 3: (2,-1)
    pos_x[0] = 0.0f; pos_y[0] = 0.0f;
    pos_x[1] = 5.0f; pos_y[1] = 0.0f;
    pos_x[2] = 2.0f; pos_y[2] = 2.0f;
    pos_x[3] = 2.0f; pos_y[3] = -1.0f;
    pos_init = 1;
}

static float compute_range_m(int a, int b) {
    float dx = pos_x[a] - pos_x[b];
    float dy = pos_y[a] - pos_y[b];
    float d2 = dx*dx + dy*dy;
    return sqrtf(d2);
}

static void store_range(Node* tag, int anchor_id, float range_m, unsigned long now_ms) {
    // Update existing entry if present
    for (int i = 0; i < tag->range_count; i++) {
        if (tag->ranges[i].anchor_id == anchor_id) {
            tag->ranges[i].last_range_m = range_m;
            tag->ranges[i].last_update_ms = now_ms;
            return;
        }
    }
    // Append if space
    if (tag->range_count < MAX_NEIGHBORS) {
        tag->ranges[tag->range_count].anchor_id = anchor_id;
        tag->ranges[tag->range_count].last_range_m = range_m;
        tag->ranges[tag->range_count].last_update_ms = now_ms;
        tag->range_count++;
    }
}

int enqueue_msg(MessageQueue *q, Message *msg) {
    int next_tail = (q->tail + 1) % MAX_MESSAGES;
    if (next_tail == q->head) return 0;  // Queue full
    q->queue[q->tail] = *msg;
    q->tail = next_tail;
    return 1;
}

int dequeue_msg(MessageQueue *q, Message *msg) {
    if (q->head == q->tail) return 0;  // Queue empty
    *msg = q->queue[q->head];
    q->head = (q->head + 1) % MAX_MESSAGES;
    return 1;
}

// Check if to_id is a neighbor of from_id
int is_reachable(int from_id, int to_id) {
    NeighborTable *table = &neighbor_tables[from_id];
    for (int i = 0; i < table->neighbor_count; i++) {
        if (table->neighbors[i] == to_id) return 1;
    }
    return 0;
}

// Add/update/prune neighbors
void update_neighbor(int node_id, int neighbor_id) {
    if (node_id == neighbor_id || node_id < 0 || neighbor_id < 0) return;
    NeighborTable *table = &neighbor_tables[node_id];
    unsigned long now = get_current_time_ms();
    // Check if already known neighbor
    for (int i = 0; i < table->neighbor_count; i++) {
        if (table->neighbors[i] == neighbor_id) {
            table->last_seen[i] = now;
            return;
        }
    }
    // Add new neighbor if space
    if (table->neighbor_count < MAX_NEIGHBORS) {
        table->neighbors[table->neighbor_count] = neighbor_id;
        table->last_seen[table->neighbor_count] = now;
        table->neighbor_count++;
    }
}
void prune_neighbors(int node_id) {
    NeighborTable *table = &neighbor_tables[node_id];
    unsigned long now = get_current_time_ms();
    for (int i = 0; i < table->neighbor_count; ) {
        if (now - table->last_seen[i] > NEIGHBOR_TIMEOUT_MS) {
            // Shift left to remove expired neighbor
            for (int j = i; j < table->neighbor_count - 1; j++) {
                table->neighbors[j] = table->neighbors[j + 1];
                table->last_seen[j] = table->last_seen[j + 1];
            }
            table->neighbor_count--;
        } else {
            i++;
        }
    }
}

void send_message(int from_id, int to_id, MsgType type) {
    if (from_id < 0 && from_id != -1) return;
    if (to_id < 0) return;

    // TDMA: require reachability for data-plane messages
    if (from_id != -1 && !is_reachable(from_id, to_id)) return;

    // For real-node traffic, update neighbor info (keepalive)
    if (from_id != -1) {
        update_neighbor(from_id, to_id);
        update_neighbor(to_id, from_id);
    }

    Message msg = (Message){ type, from_id, to_id, 0 };
    // Delegate actual delivery/logging to the simulation's PHY
    phy_unicast(&msg);
}

// Schedule a message to be sent by this node after delay_ms
static void schedule_tx(Node* node, const Message* m, unsigned long delay_ms) {
    if (!node || !m) return;
    for (int i = 0; i < MAX_MESSAGES; i++) {
        if (!node->txq[i].in_use) {
            node->txq[i].msg = *m;
            node->txq[i].due_ms = get_current_time_ms() + delay_ms;
            node->txq[i].in_use = 1;
            return;
        }
    }
    // Drop if TX queue full
}

// Flush all due transmissions for this node
static void flush_due_tx(Node* node) {
    if (!node) return;
    unsigned long now = get_current_time_ms();
    for (int i = 0; i < MAX_MESSAGES; i++) {
        if (node->txq[i].in_use && node->txq[i].due_ms <= now) {
            Message* m = &node->txq[i].msg;
            // Data-plane goes through send_message (reachability checks + keepalive)
            if (m->type == MSG_TWR_REQUEST || m->type == MSG_TWR_RESPONSE) {
                send_message(m->from_id, m->to_id, m->type);
            } else {
                // Control-plane uses PHY directly (keep TTL intact for BEACON)
                phy_unicast(m);
            }
            node->txq[i].in_use = 0;
        }
    }
}

// Emit a BEACON from a coordinator anchor; anchors relay with ttl-1.
void emit_beacon(int origin_anchor_id, int ttl) {
    if (origin_anchor_id < 0 || origin_anchor_id >= MAX_NODES) return;
    if (!nodes[origin_anchor_id] || nodes[origin_anchor_id]->role != ANCHOR) return;
    if (ttl <= 0) return;

    NeighborTable *table = &neighbor_tables[origin_anchor_id];
    for (int i = 0; i < table->neighbor_count; i++) {
        int nid = table->neighbors[i];
        if (nid < 0 || nid >= MAX_NODES || !nodes[nid]) continue;

        Message m = (Message){ MSG_BEACON, origin_anchor_id, nid, ttl };
        phy_unicast(&m);
    }
}

// Anchors: respond to TWR and relay BEACONs with TTL.
// Also periodically ANNOUNCE for discovery.
void process_anchor(Node *node) {
    prune_neighbors(node->id);

    // Flush any due transmissions first (node-owned scheduling)
    flush_due_tx(node);

    // Periodic announce for neighbor discovery
    unsigned long now = get_current_time_ms();
    if (now - node->last_announce_ms >= ANNOUNCE_PERIOD_MS) {
        node->last_announce_ms = now;
        phy_broadcast(node->id, MSG_ANNOUNCE, 0);
    }

    Message msg;
    while (dequeue_msg(&node->msg_queue, &msg)) {
        // Notify simulation layer (receiving)
        on_message_received(&msg, node->id);
        // VCD: mark RX at this node
        trace_phy_rx(&msg, node->id);

        update_neighbor(node->id, msg.from_id);

        switch (msg.type) {
            case MSG_ANNOUNCE:
                // no response needed; last_seen updated above
                break;
            case MSG_BEACON: {
                if (msg.ttl > 0) {
                    NeighborTable *t = &neighbor_tables[node->id];
                    for (int i = 0; i < t->neighbor_count; i++) {
                        int nid = t->neighbors[i];
                        if (nid == msg.from_id || !nodes[nid]) continue;
                        Message f = (Message){ MSG_BEACON, node->id, nid, msg.ttl - 1 };
                        // Schedule a slight relay delay
                        schedule_tx(node, &f, BEACON_RELAY_DELAY_MS);
                    }
                }
                break;
            }
            case MSG_TWR_REQUEST: {
                // Schedule a small processing delay before replying
                Message resp = (Message){ MSG_TWR_RESPONSE, node->id, msg.from_id, 0 };
                schedule_tx(node, &resp, ANCHOR_TWR_PROC_DELAY_MS);
                break;
            }
            default:
                break;
        }
    }
}

// Tags: range only when 'synced' and during their TDMA slot.
// Also periodically ANNOUNCE for discovery.
void process_tag(Node *node) {
    prune_neighbors(node->id);

    static int pending_twr_responses[MAX_NODES] = {0};
    static unsigned long twr_deadline_ms[MAX_NODES] = {0};

    // Flush any due transmissions first (node-owned scheduling)
    flush_due_tx(node);

    // Reset per-slot tracking when slot owner changes
    if (node->last_slot_owner != current_slot_tag_id) {
        node->last_slot_owner = current_slot_tag_id;
        node->did_range_this_slot = 0;
    }

    // Periodic announce for neighbor discovery
    unsigned long now = get_current_time_ms();
    if (now - node->last_announce_ms >= ANNOUNCE_PERIOD_MS) {
        node->last_announce_ms = now;
        phy_broadcast(node->id, MSG_ANNOUNCE, 0);
    }

    Message msg;
    while (dequeue_msg(&node->msg_queue, &msg)) {
        // Notify simulation layer (receiving)
        on_message_received(&msg, node->id);
        // VCD: mark RX at this node
        trace_phy_rx(&msg, node->id);

        update_neighbor(node->id, msg.from_id);

        switch (node->state) {
            case TAG_IDLE:
            case TAG_WAIT_FOR_SLOT:
                if (msg.type == MSG_BEACON) {
                    node->synced = 1;
                    node->last_beacon_ms = get_current_time_ms();
                }
                break;
            case TAG_INITIATE_TWR:
                break;
            case TAG_WAIT_FOR_TWR_RESPONSES:
                if (msg.type == MSG_TWR_RESPONSE) {
                    ensure_positions(node->id, TAG);
                    float rng = compute_range_m(node->id, msg.from_id);
                    store_range(node, msg.from_id, rng, get_current_time_ms());
                    if (pending_twr_responses[node->id] > 0) {
                        pending_twr_responses[node->id]--;
                        if (pending_twr_responses[node->id] == 0) {
                            node->state = TAG_END_SLOT;
                        }
                    }
                } else if (msg.type == MSG_BEACON) {
                    node->synced = 1;
                    node->last_beacon_ms = get_current_time_ms();
                }
                break;
            case TAG_END_SLOT:
                break;
            default:
                break;
        }
    }

    // Enforce sync validity
    now = get_current_time_ms();
    if (node->synced && (now - node->last_beacon_ms) > SYNC_TIMEOUT_MS) {
        node->synced = 0;
        if (node->state != TAG_WAIT_FOR_TWR_RESPONSES) {
            node->state = TAG_WAIT_FOR_SLOT;
        }
    }

    // TDMA gating
    if ((node->state == TAG_WAIT_FOR_SLOT || node->state == TAG_IDLE) &&
        node->synced && current_slot_tag_id == node->id &&
        node->did_range_this_slot == 0) {
        node->state = TAG_INITIATE_TWR;
        node->did_range_this_slot = 1;
    }

    // Timeout
    if (node->state == TAG_WAIT_FOR_TWR_RESPONSES &&
        pending_twr_responses[node->id] > 0 &&
        now >= twr_deadline_ms[node->id]) {
        pending_twr_responses[node->id] = 0;
        node->state = TAG_END_SLOT;
    }

    // One-shot actions
    switch (node->state) {
        case TAG_INITIATE_TWR: {
            ensure_positions(node->id, TAG);
            int twr_count = 0;
            NeighborTable *table = &neighbor_tables[node->id];
            for (int i = 0; i < table->neighbor_count; i++) {
                int neighbor_id = table->neighbors[i];
                if (nodes[neighbor_id]->role == ANCHOR) {
                    // Send immediately (data-plane paths still see 1-tick air latency)
                    send_message(node->id, neighbor_id, MSG_TWR_REQUEST);
                    twr_count++;
                }
            }
            pending_twr_responses[node->id] = twr_count;
            if (twr_count == 0) {
                node->state = TAG_END_SLOT;
            } else {
                twr_deadline_ms[node->id] = get_current_time_ms() + TWR_RESPONSE_TIMEOUT_MS;
                node->state = TAG_WAIT_FOR_TWR_RESPONSES;
            }
            break;
        }
        case TAG_END_SLOT: {
            node->state = TAG_IDLE;
            break;
        }
        default:
            break;
    }
}

void add_node(int id, Role role, void (*process_func)(Node *)) {
    if (node_count >= MAX_NODES) {
        printf("Max nodes reached, can't add more.\n");
        return;
    }
    Node *node = malloc(sizeof(Node));
    node->id = id;
    node->role = role;
    node->msg_queue.head = 0;
    node->msg_queue.tail = 0;
    node->next_queue.head = 0;   // init next-step delivery queue
    node->next_queue.tail = 0;
    node->process = process_func;
    node->synced = 0;
    node->last_beacon_ms = 0;
    node->range_count = 0;
    node->last_slot_owner = -2;     // force a reset on first observation
    node->did_range_this_slot = 0;  // not ranged yet in current (future) slot
    if (role == TAG) {
        node->state = TAG_WAIT_FOR_SLOT; // TDMA: waiting for slot
    } else {
        node->state = ANCHOR_IDLE;
    }
    nodes[node_count++] = node;

    neighbor_tables[id].neighbor_count = 0;

    // Initialize simple geometry once
    ensure_positions(id, role);

    // Initialize discovery timer
    nodes[node_count - 1]->last_announce_ms = 0;

    // Initialize delayed TX queue
    for (int i = 0; i < MAX_MESSAGES; i++) {
        node->txq[i].in_use = 0;
        node->txq[i].due_ms = 0;
    }
}

void add_neighbor(int node_id, int neighbor_id) {
    if (node_id < 0 || neighbor_id < 0 || node_id >= MAX_NODES || neighbor_id >= MAX_NODES) return;
    NeighborTable *table = &neighbor_tables[node_id];
    if (table->neighbor_count < MAX_NEIGHBORS) {
        for (int i = 0; i < table->neighbor_count; i++) {
            if (table->neighbors[i] == neighbor_id) return;
        }
        table->neighbors[table->neighbor_count] = neighbor_id;
        table->last_seen[table->neighbor_count] = get_current_time_ms();
        table->neighbor_count++;
    }
}

// Helper: find grid position for a node id; returns 1 if found
static int find_node_in_grid(int id, int* out_x, int* out_y) {
    for (int y = 0; y < PHY_GRID_H; y++) {
        for (int x = 0; x < PHY_GRID_W; x++) {
            int v = PHY_GRID[y][x];
            if (v > 0 && (v - 1) == id) {
                *out_x = x; *out_y = y;
                return 1;
            }
        }
    }
    return 0;
}

// Public API: search for neighbors based on Manhattan distance over the global grid.
// Updates only this node's neighbor table (no symmetry assumed here).
void discover_neighbors(int node_id, int dist_thresh) {
    if (node_id < 0 || node_id >= MAX_NODES) return;

    int xA = -1, yA = -1;
    if (!find_node_in_grid(node_id, &xA, &yA)) return;

    for (int y = 0; y < PHY_GRID_H; y++) {
        for (int x = 0; x < PHY_GRID_W; x++) {
            int v = PHY_GRID[y][x];
            if (v <= 0) continue;
            int other_id = v - 1;
            if (other_id == node_id) continue;

            int dx = xA - x; if (dx < 0) dx = -dx;
            int dy = yA - y; if (dy < 0) dy = -dy;
            int manhattan = dx + dy;
            if (manhattan <= dist_thresh) {
                add_neighbor(node_id, other_id);
            }
        }
    }
}
