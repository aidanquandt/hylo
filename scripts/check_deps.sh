#!/usr/bin/env bash
# Verify build dependencies. Exit 0 if all present, 1 otherwise.
# Used by: make check-deps

set -e
MISSING=0

check() {
  if command -v "$1" &>/dev/null; then
    echo "  OK $1: $(command -v "$1")"
  else
    echo "  MISSING $1" >&2
    MISSING=1
  fi
}

echo "Build dependencies:"
check cmake
check ninja
check arm-none-eabi-gcc

if [[ $MISSING -ne 0 ]]; then
  echo "" >&2
  echo "Install missing tools (e.g. MSYS2 UCRT64: pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-gcc cmake ninja)" >&2
  exit 1
fi
