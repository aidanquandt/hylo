#!/usr/bin/env bash
# Flash firmware to STM32H723 via OpenOCD (ST-Link).
# Builds debug elf only if missing.
# Run from repo root. Usage: flash.sh <REV> where REV is 0 or 1

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT_DIR"

REV="${1:-}"
if [[ -z "$REV" ]]; then
    echo "Error: REV argument required (0 or 1)" >&2
    exit 1
fi
if [[ "$REV" != "0" ]] && [[ "$REV" != "1" ]]; then
    echo "Error: REV must be 0 or 1 (got: $REV)" >&2
    exit 1
fi

ELF="build/arm-gcc-debug/hwconfig/stm32_hwrev_${REV}/stm32_hwrev_${REV}.elf"

if [[ ! -f "$ELF" ]]; then
    echo "No build found for hwrev ${REV}; building debug first..."
    make build REV="$REV"
fi

if [[ ! -f "$ELF" ]]; then
    echo "Build artifact not found: $ELF" >&2
    exit 1
fi

echo "=== Flashing STM32H723 (hwrev ${REV}) ==="
echo

"$SCRIPT_DIR/attach-usb.sh" stlink

OPENOCD_CFG="$ROOT_DIR/tools/openocd/openocd.cfg"

if command -v openocd &> /dev/null; then
    openocd -f "$OPENOCD_CFG" \
        -c "program $ELF verify" \
        -c "reset run" \
        -c "exit"

    echo
    echo "Flash complete"
else
    echo "Error: OpenOCD not found!"
    echo "  Ubuntu/WSL:  sudo apt install openocd"
    exit 1
fi
