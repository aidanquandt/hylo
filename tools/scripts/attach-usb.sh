#!/bin/bash
# Ensure a USB device is attached to WSL for flashing or serial tools.
# Invokes ~/.config/wsl-usb/attach.sh if present (WSL2 + usbipd setup).
# No-op on native Linux or when wsl-usb is not installed.
# Usage: attach-usb.sh [stlink|serial]

set -euo pipefail

ATTACH_SCRIPT="${HOME}/.config/wsl-usb/attach.sh"
DEVICE="${1:-stlink}"

if [[ -f "$ATTACH_SCRIPT" ]]; then
    "$ATTACH_SCRIPT" "$DEVICE" 2>/dev/null || true
fi
