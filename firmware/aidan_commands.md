# Aidan Commands Reference

## Table of Contents
- [Quick Start: Anchor and Tag](#quick-start-anchor-and-tag)
- [TWR (Two-Way Ranging)](#twr-two-way-ranging)
- [TWR Auto-Ranging with Multi-Target Scheduler](#twr-auto-ranging-with-multi-target-scheduler)
- [Quick Setup](#quick-setup)
- [Remote Configuration (OTA Programming)](#remote-configuration-ota-programming)
- [UWB Port Stats](#uwb-port-stats)
- [Beacon Send](#beacon-send)
- [Stopwatch](#stopwatch)
- [Datalogger](#datalogger)

---

## Quick Start: Anchor and Tag

Set up a basic anchor and tag for ranging. Use these commands frequently for quick testing:

```bash
uwb_node.set.type tag
uwb_node.set.address 0x0002
ota_config.send.address 0x0000 0x0001
twrmgr.add.target 0x0001
twrmgr.req.start

ota_config.send.position 0x0001 6.0 0.0 2.5

twrmgr.req.stop


beacon.req.ping 0x0001
beacon.req.ping 0x0002
beacon.req.ping 0x0003
```

---

## TWR (Two-Way Ranging)
*Simplified Architecture - No Mode Setting Needed*

### Setup Responder (Fixed/Anchor Device)
Both roles always active:
```bash
uwb_node.set.type anchor
uwb_node.set.address 0x0001
uwb_node.set.position 0.0 0.0 2.5
```

**Note:** 
- `uwb_node.set.address` updates the DW3000 address (UWB module is single source of truth)
- Responder automatically restarted when address changes
- Responder automatically listens for incoming POLL messages

### Setup Initiator (Mobile/Tag Device)
Both roles always active:
```bash
uwb_node.set.type tag
uwb_node.set.address 0x0002
```

**Note:** Can now both initiate ranging AND respond to incoming POLLs

### Single Range
Any node can initiate:
```bash
twr.req.range 0x0001
twr.get.status  # Shows both initiator and responder state
```

---

## TWR Auto-Ranging with Multi-Target Scheduler

### Setup Responder Side (Fixed Infrastructure)
```bash
uwb_node.set.type anchor
uwb_node.set.address 0x0001
uwb_node.set.position 0.0 0.0 2.5
```

### Setup Initiator Side (Mobile Device)
```bash
uwb_node.set.type tag
uwb_node.set.address 0x0002
```

### ⚠️ Configure Targets
**IMPORTANT:** Configure targets BEFORE starting manager!

#### Option 1: Add Targets One by One
```bash
twrmgr.add.target 0x0001
twrmgr.add.target 0x0002
twrmgr.add.target 0x0003
```

#### Option 2: Set All at Once (Replaces Existing List)
```bash
twrmgr.set.targets 0x0001 0x0002 0x0003
```

### Start Ranging
Will cycle through: 0x0001 → 0x0002 → 0x0003 → 0x0001...
```bash
twrmgr.req.start
twrmgr.get.status
twrmgr.req.stop
```

### Remove a Target from Schedule
```bash
twrmgr.remove.target 0x0002
```

### Aidan Default Setup
```bash
uwb_node.set.type tag
uwb_node.set.address 0x0002
twrmgr.add.target 0x0001
twrmgr.req.start
```

### Check Status
```bash
twrmgr.get.status
```

---

## Quick Setup

UWB module is the single source of truth for addressing. Node provides convenient API:

```bash
uwb_node.set.address 0x0001
twrmgr.req.start
```

---

## Remote Configuration (OTA Programming)

Program node addresses, positions, and types over UWB without needing UART access to each device.

### Security Setup

**Set authentication token** (both sending and receiving nodes must use same token):
```bash
ota_config.set.token 0xDECABEEF    # Default token
ota_config.get.token               # View current token
```

### Program Remote Node Address

Change a remote node's UWB address:
```bash
# Change node 0x0001 to new address 0x0002
ota_config.send.address 0x0001 0x0002

# Change address and PAN ID
ota_config.send.address 0x0001 0x0002 0xABCD
```

**⚠️ Note:** After address change, node only responds to NEW address!

### Program Remote Node Position

Set anchor position coordinates (x, y, z in meters):
```bash
# Set node 0x0002 at position (5.0, 10.0, 2.5)
ota_config.send.position 0x0002 5.0 10.0 2.5

# Configure 4 corner anchors
ota_config.send.position 0x0010 0.0 0.0 2.5    # Bottom-left
ota_config.send.position 0x0011 10.0 0.0 2.5   # Bottom-right
ota_config.send.position 0x0012 10.0 8.0 2.5   # Top-right
ota_config.send.position 0x0013 0.0 8.0 2.5    # Top-left
```

### Program Remote Node Type

Set node type (0=TAG, 1=ANCHOR, 2=HYBRID):
```bash
ota_config.send.type 0x0002 1      # Configure as ANCHOR
ota_config.send.type 0x0003 0      # Configure as TAG
```

### Control Remote GPIO/LED

Control GPIO pins (e.g., LED) on remote nodes:
```bash
# Turn LED ON on node 0x0002
ota_config.set.gpio 0x0002 0 1

# Turn LED OFF on node 0x0002
ota_config.set.gpio 0x0002 0 0
```

**Parameters:**
- `target_addr`: Target node address (hex)
- `pin`: GPIO pin (0 = LED_GREEN)
- `state`: 0 = OFF/LOW, 1 = ON/HIGH

**Use Cases:**
- Visual identification of nodes in field
- Signal node status or mode
- Debug/testing aid

### Complete Example: Setup 4 Anchors

Program anchors at room corners without UART access:
```bash
# Set auth token on master node
ota_config.set.token 0xDECABEEF

# Anchor 1 (0x0001) -> Bottom-left
ota_config.send.address 0x0001 0x0010
ota_config.send.type 0x0010 1
ota_config.send.position 0x0010 0.0 0.0 2.5

# Anchor 2 (0x0002) -> Bottom-right
ota_config.send.address 0x0002 0x0011
ota_config.send.type 0x0011 1
ota_config.send.position 0x0011 10.0 0.0 2.5

# Anchor 3 (0x0003) -> Top-right
ota_config.send.address 0x0003 0x0012
ota_config.send.type 0x0012 1
ota_config.send.position 0x0012 10.0 8.0 2.5

# Anchor 4 (0x0004) -> Top-left
ota_config.send.address 0x0004 0x0013
ota_config.send.type 0x0013 1
ota_config.send.position 0x0013 0.0 8.0 2.5
```

### View Statistics

Check configuration operations:
```bash
ota_config.get.stats
```
Shows:
- Requests sent
- Responses received
- Authentication failures

**Troubleshooting:**
- If auth failures increase → check tokens match on both nodes
- If no response → verify target address and UWB range
- Target node logs all config changes via error handler

---

## UWB Port Stats

```bash
uwb.get.stats
```

---

## Beacon Send

### Setup Node 1 and Ping Node 2
```bash
uwb_node.set.address 0x0001
beacon.req.ping 0x0002
```

### Setup Node 2 and Ping Node 1
```bash
uwb_node.set.address 0x0002
beacon.req.ping 0x0001
```

### Delayed Ping
```bash
uwb_node.set.address 0x0002
beacon.req.ping.delayed 0x0001
```

---

## Stopwatch

```bash
stopwatch.get.0
stopwatch.get.1
stopwatch.get.all
```

---

## Datalogger

```bash
datalogger.get.stats
```
