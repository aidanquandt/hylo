# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Hylo** is a UWB (Ultra-Wideband) hybrid localization system. The firmware runs on STM32H723 microcontrollers and implements time-windowed ranging (TWR), IMU fusion, and telemetry over UART/WiFi. Host-side Python tools handle visualization, serial communication, and a web interface.

## Build Commands

```bash
# Check toolchain dependencies
make check-deps

# Initialize nanopb submodule (required before first build)
git submodule update --init third_party/nanopb

# Build firmware (REV=0: Nucleo dev board, REV=1: custom PCB)
make build REV=0
make build REV=1

# Flash firmware via OpenOCD + ST-Link
make flash REV=0

# Protocol code generation (auto-runs if .proto files are newer than generated output)
make protocol-codegen

# Verify generated protocol code
python tools/protocol_codegen/verify_codegen.py
```

## Host Tools

```bash
make host-interactive PORT=<PORT>   # Protocol REPL (send/receive messages)
make host-monitor                    # Raw serial monitor
make host-webapp                     # FastAPI server at http://127.0.0.1:8000
make host-visualization              # IMU 3D attitude display (requires pygame, PyOpenGL)
make host-enable-streaming           # Enable IMU streaming on device
```

## Architecture

The firmware uses a strict layered dependency model — **app code must never directly access HAL or third-party vendor code**:

```
hwconfig (STM32CubeMX HAL) → drivers/ (HAL wrapper)
                                     ↓
                                   src/ (app modules)
                                     ↓
                    lib/ (utilities) + protocol/ (nanopb messaging)
```

### Key Directories

- **`hwconfig/`** — Per-revision hardware configs (STM32CubeMX generated files, linker scripts). `hwrev_0` = Nucleo board, `hwrev_1` = custom PCB.
- **`drivers/`** — Sole interface between app and HAL. Contains: uart, spi, i2c, gpio, timer, imu, uwb, eeprom, sdcard, system, os, watchdog.
- **`src/`** — Application modules (flat layout, one directory per module):
  - `twr/` — Time-windowed ranging state machine and algorithm
  - `twr_manager/` — TWR scheduling and multi-node coordination
  - `uwb/` — UWB MAC layer, device init, RX/TX queues
  - `uart_protocol/` — COBS + CRC16 framing, protocol RX dispatcher
  - `sensor_fusion/` — Kalman filter based IMU fusion (CMSIS-DSP)
  - `datalogger/` — SD card and memory logging
  - `wifi/` — WiFi and OTA updates
- **`lib/`** — Reusable utilities with minimal dependencies: backoff, counter, state_machine, stopwatch, mac_802154, uwb_protocol.
- **`protocol/`** — `.proto` definitions; codegen produces `generated/protocol/`.
- **`common/`** — Global headers: `feature_config.h` (feature toggles), `task_config.h` (task stack sizes/priorities).
- **`third_party/`** — Vendored: dwt_uwb_driver (UWB chip), bmi323 (IMU), nanopb (Protocol Buffers).
- **`host/`** — Python host tools: serial REPL, FastAPI webapp, visualization.

### Module System

App modules implement optional callbacks registered in `src/module/module.h` and scheduled in `src/app/app.c`:
- `module_init()` — One-time initialization
- `module_create_task()` — Create FreeRTOS task
- `module_process_1hz/10hz/100hz/1khz()` — Periodic processing

### Protocol System

All firmware↔host communication uses Protocol Buffers (nanopb). To modify:
1. Edit `protocol/*.proto`
2. Run `make protocol-codegen` (or it auto-runs on `make build` if needed)
3. Generated outputs: `generated/protocol/c/` (C) and `generated/protocol/python/` (Python)

## Code Style

- **Braces:** Allman style (braces on their own line)
- **Indentation:** 4 spaces
- **Naming:** `snake_case` for variables/functions, CamelCase for typedef'd types
- **Column limit:** 100 characters
- **Formatter:** clang-format (config in `.clang-format`)

## Configuration

- `common/feature_config.h` — Feature toggles (TWR, sensor fusion, logging, WiFi, etc.)
- `src/wifi/wifi_config.h` — WiFi credentials (auto-copied from `.example` on first build; gitignored)
- Hardware revision selected at build time via `REV=` make variable or `-DHW_REV=` cmake flag

## CMake Presets

- `arm-gcc-debug` — Debug build with `-g`, outputs to `build/arm-gcc-debug/`
- `arm-gcc-release` — Release build with `-O2`, outputs to `build/arm-gcc-release/`
