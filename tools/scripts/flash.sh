#!/usr/bin/env bash
# Flash firmware to device via OpenOCD (ST-Link). Builds debug elf only if missing.
# Run from repo root. Requires: openocd, arm-none-eabi toolchain (for build).
# Usage: flash.sh <REV> where REV is 0 or 1

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

# Get REV from argument
REV="${1:-}"
if [[ -z "$REV" ]]; then
  echo "Error: REV argument required (0 or 1)" >&2
  exit 1
fi

if [[ "$REV" != "0" ]] && [[ "$REV" != "1" ]]; then
  echo "Error: REV must be 0 or 1 (got: $REV)" >&2
  exit 1
fi

# Build only if artifact is missing
ELF_REL="build/arm-gcc-debug/hwconfig/stm32_hwrev_${REV}/stm32_hwrev_${REV}.elf"
if [[ ! -f "$ELF_REL" ]]; then
  echo "No build found for hwrev ${REV}; building debug first..."
  make build REV="$REV"
fi

# Use relative path so OpenOCD (Windows) can open the file when run from MSYS2/Git Bash
if [[ ! -f "$ELF_REL" ]]; then
  echo "Build artifact not found: $ELF_REL" >&2
  exit 1
fi

openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "program $ELF_REL verify reset exit"
