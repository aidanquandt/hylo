#!/usr/bin/env bash
# Verify project dependencies. Exit 0 if all *build* deps present, 1 otherwise.
# Also reports protocol-codegen, flash, and host-tool deps (missing ones do not change exit code).
# Used by: make check-deps
# Full list: tools/scripts/DEPENDENCIES.md

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

check_optional() {
  if command -v "$1" &>/dev/null; then
    echo "  OK $1: $(command -v "$1")"
  else
    echo "  (optional) $1 not found"
  fi
}

# Optional Python package (host tools, webapp, visualization)
check_py_optional() {
  local pkg="$1"
  local name="${2:-$1}"
  if python3 -c "import $pkg" 2>/dev/null; then
    echo "  OK $name (python3)"
  else
    echo "  (optional) $name not found (pip install $pkg)"
  fi
}

echo "--- Build (required for make build / make release) ---"
check cmake
check ninja
check arm-none-eabi-gcc

echo ""
echo "--- Protocol codegen (required for make protocol-codegen) ---"
check_optional python3
check_optional protoc

echo ""
echo "--- Flash (required for make flash) ---"
check_optional openocd

echo ""
echo "--- Host tools (Python) ---"
if command -v python3 &>/dev/null; then
  check_py_optional serial "pyserial"
  check_py_optional fastapi "fastapi"
  check_py_optional uvicorn "uvicorn"
  check_py_optional pydantic "pydantic"
  check_py_optional pygame "pygame"
  check_py_optional OpenGL "PyOpenGL"
else
  echo "  (skip Python packages: python3 not found)"
fi

if [[ $MISSING -ne 0 ]]; then
  echo "" >&2
  echo "Install missing build tools (e.g. MSYS2 UCRT64: pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-gcc cmake ninja)" >&2
  exit 1
fi
