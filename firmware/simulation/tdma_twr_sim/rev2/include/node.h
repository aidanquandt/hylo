#ifndef NODE_H
#define NODE_H

#define MAX_NODES 10
#define MAX_MESSAGES 10
#define MAX_NEIGHBORS 5

// Periodic announce interval for neighbor discovery (ms, sim-time)
#define ANNOUNCE_PERIOD_MS 1000
// Global simulation step (ms) used by the simulator to advance time deterministically
// Set to 1 ms so VCD timescale is 1 ms and timebase has ms resolution.
#define SIM_STEP_MS 1

typedef enum { ANCHOR, TAG } Role;

typedef enum {
    // Periodic beacon for neighbor discovery/keepalive; refreshes last_seen. No response required.
    MSG_ANNOUNCE,
    // Network-wide time/schedule beacon; flooded via anchors with TTL.
    // Tags mark themselves 'synced' upon receipt and are allowed to range only while synced.
    MSG_BEACON,
    // Tag -> Anchor: initiate a two-way ranging (TWR) exchange with an anchor.
    MSG_TWR_REQUEST,
    // Anchor -> Tag: reply to a TWR request; completes one TWR exchange for that anchor.
    MSG_TWR_RESPONSE
} MsgType;

typedef struct {
    MsgType type;
    int from_id;
    int to_id;
    int ttl;       // Hop limit for flooded control messages (e.g., BEACON); 0 for data-plane.
} Message;

typedef struct Node Node;

typedef struct {
    Message queue[MAX_MESSAGES];
    int head;
    int tail;
} MessageQueue;

typedef struct {
    int neighbors[MAX_NEIGHBORS];
    int neighbor_count;
    unsigned long last_seen[MAX_NEIGHBORS]; // last heard timestamps
} NeighborTable;

// Simple per-anchor ranging result stored by tags
typedef struct {
    int anchor_id;                // which anchor this entry corresponds to
    float last_range_m;           // most recent computed range (meters) to this anchor
    unsigned long last_update_ms; // sim-time when last_range_m was updated
} RangingEntry;

// Tag states (TDMA)
// - Tags may initiate TWR only during their TDMA slot (TAG_INITIATE_TWR path) AND while 'synced'.
// - 'synced' is asserted by receipt of a BEACON and expires after a timeout, forcing re-sync.
typedef enum {
    TAG_IDLE,
    TAG_WAIT_FOR_SLOT,
    TAG_INITIATE_TWR,
    TAG_WAIT_FOR_TWR_RESPONSES,
    TAG_END_SLOT
} TagState;

// Anchor states
typedef enum {
    ANCHOR_IDLE
} AnchorState;

typedef struct {
    Message msg;               // full message (type/from/to/ttl)
    unsigned long due_ms;      // when to transmit
    int in_use;                // slot occupancy
} PendingTx;

struct Node {
    int id;
    Role role;
    MessageQueue msg_queue;     // current inbox (delivered this step)
    MessageQueue next_queue;    // simulator's "next-step" delivery queue
    void (*process)(Node *node);
    int state; // Use TagState or AnchorState depending on role

    // Runtime fields primarily used by tags (anchors ignore)
    int synced;                    // set when a BEACON is received, cleared on timeout
    unsigned long last_beacon_ms;  // last BEACON reception time
    RangingEntry ranges[MAX_NEIGHBORS]; // most recent range per visible anchor neighbor
    int range_count;               // number of valid entries in ranges[]

    // TDMA slot tracking to avoid duplicate initiations within a single slot
    int last_slot_owner;           // last observed current_slot_tag_id
    int did_range_this_slot;       // 1 once TWR was initiated in the current slot

    // Discovery
    unsigned long last_announce_ms; // last time an ANNOUNCE was sent

    // Outgoing delayed transmissions (node-owned scheduling)
    PendingTx txq[MAX_MESSAGES];
};

// Message queue functions
int enqueue_msg(MessageQueue *q, Message *msg);
int dequeue_msg(MessageQueue *q, Message *msg);

// Send message simulator
void send_message(int from_id, int to_id, MsgType type);

// Emit a BEACON from a coordinator anchor with a given TTL (anchors will relay).
void emit_beacon(int origin_anchor_id, int ttl);

// Implement this to get platform/system time in ms
unsigned long get_current_time_ms();

// Global simulation time (ms) and simulator-side advance hook
extern unsigned long g_sim_time_ms;
void sim_advance_time_ms(unsigned long delta_ms);

// Monitoring hooks implemented in simulation.c
void on_message_sent(const Message* msg);
void on_message_received(const Message* msg, int receiver_id);

// PHY hooks provided by the simulation layer
void phy_unicast(const Message* msg);
void phy_broadcast(int from_id, MsgType type, int ttl);

// Node process functions
void process_anchor(Node *node);
void process_tag(Node *node);

// Add a new node to simulation
void add_node(int id, Role role, void (*process_func)(Node *));

// Neighbor table utilities
extern Node *nodes[MAX_NODES];
extern int node_count;
extern NeighborTable neighbor_tables[MAX_NODES];

void add_neighbor(int node_id, int neighbor_id);
void prune_neighbors(int node_id);
void update_neighbor(int node_id, int neighbor_id);

// TDMA: current active tag slot (set by simulation.c each round)
// Only the tag that owns the slot AND is 'synced' may start TWR this step.
extern int current_slot_tag_id;

// Physical grid (global, read-only for nodes)
// 0 = empty; v > 0 places node with id = v - 1 at that cell.
extern int PHY_GRID_H;
extern int PHY_GRID_W;
extern const int PHY_GRID[10][10];

// Neighbor discovery (Manhattan distance threshold in grid cells)
void discover_neighbors(int node_id, int dist_thresh);

// PHY trace hooks (implemented in trace_vcd.c)
void trace_phy_tx(const Message* msg);
void trace_phy_rx(const Message* msg, int receiver_id);

#endif /* NODE_H */
