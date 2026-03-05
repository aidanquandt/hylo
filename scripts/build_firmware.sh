#!/usr/bin/env bash
# Firmware build script. Usage: ./scripts/build_firmware.sh [debug|release]
# Default: debug. Run from repo root (called by scripts/build.sh).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

CONFIG="${1:-debug}"

case "$CONFIG" in
  debug)
    cmake --preset=arm-gcc-debug && cmake --build --preset=arm-gcc-debug
    ;;
  release)
    cmake --preset=arm-gcc-release && cmake --build --preset=arm-gcc-release
    ;;
  *)
    echo "Unknown config: $CONFIG (use debug|release)" >&2
    exit 1
    ;;
esac
