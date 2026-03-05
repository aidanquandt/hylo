#!/usr/bin/env bash
# Flash firmware to device via OpenOCD (ST-Link). Builds debug elf only if missing.
# Run from repo root. Requires: openocd, arm-none-eabi toolchain (for build).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

# Build only if artifact is missing
ELF_REL="build/arm-gcc-debug/hwconfig/stm32_hwrev_0/stm32_hwrev_0.elf"
if [[ ! -f "$ELF_REL" ]]; then
  echo "No build found; building debug first..."
  "$SCRIPT_DIR/build_firmware.sh" debug
fi

# Use relative path so OpenOCD (Windows) can open the file when run from MSYS2/Git Bash
if [[ ! -f "$ELF_REL" ]]; then
  echo "Build artifact not found: $ELF_REL" >&2
  exit 1
fi

openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program $ELF_REL verify reset exit"
