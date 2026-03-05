#!/usr/bin/env bash
# Build from repo root. Usage: ./scripts/build.sh [debug|release]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/build_firmware.sh" "${1:-debug}"
