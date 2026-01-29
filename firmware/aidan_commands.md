# Aidan Commands Reference

## Table of Contents
- [Quick Start: Anchor and Tag](#quick-start-anchor-and-tag)
- [TWR (Two-Way Ranging)](#twr-two-way-ranging)
- [TWR Auto-Ranging with Multi-Target Scheduler](#twr-auto-ranging-with-multi-target-scheduler)
- [Quick Setup](#quick-setup)
- [UWB Port Stats](#uwb-port-stats)
- [Beacon Send](#beacon-send)
- [Stopwatch](#stopwatch)
- [Datalogger](#datalogger)

---

## Quick Start: Anchor and Tag

Set up a basic anchor and tag for ranging. Use these commands frequently for quick testing:

```bash
node.set.type anchor
node.set.address 0x0001
node.set.position 0.0 0.0 2.5

node.set.type tag
node.set.address 0x0002
twrmgr.add.target 0x0001
twrmgr.req.start

twrmgr.req.stop
```

---

## TWR (Two-Way Ranging)
*Simplified Architecture - No Mode Setting Needed*

### Setup Responder (Fixed/Anchor Device)
Both roles always active:
```bash
node.set.type anchor
node.set.address 0x0001
node.set.position 0.0 0.0 2.5
```

**Note:** 
- `node.set.address` updates the DW3000 address (UWB module is single source of truth)
- Responder automatically restarted when address changes
- Responder automatically listens for incoming POLL messages

### Setup Initiator (Mobile/Tag Device)
Both roles always active:
```bash
node.set.type tag
node.set.address 0x0002
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
node.set.type anchor
node.set.address 0x0001
node.set.position 0.0 0.0 2.5
```

### Setup Initiator Side (Mobile Device)
```bash
node.set.type tag
node.set.address 0x0002
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
node.set.type tag
node.set.address 0x0002
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
node.set.address 0x0001
twrmgr.req.start
```

---

## UWB Port Stats

```bash
uwb.get.stats
```

---

## Beacon Send

### Setup Node 1 and Ping Node 2
```bash
node.set.address 0x0001
beacon.req.ping 0x0002
```

### Setup Node 2 and Ping Node 1
```bash
node.set.address 0x0002
beacon.req.ping 0x0001
```

### Delayed Ping
```bash
node.set.address 0x0002
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
