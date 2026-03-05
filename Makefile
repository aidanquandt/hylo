# Root Makefile: build from repo root (flattened layout).
# Usage: make [target]. Run "make help" for targets.

CONFIG ?= debug
PRESET_DEBUG   := arm-gcc-debug
PRESET_RELEASE := arm-gcc-release

.PHONY: all build clean rebuild help check-deps flash distclean debug release
.PHONY: clean-debug clean-release rebuild-release
.PHONY: host host-help host-monitor host-enable-streaming host-visualization

all build debug:
	cmake --preset=$(PRESET_DEBUG) && cmake --build --preset=$(PRESET_DEBUG)

release:
	cmake --preset=$(PRESET_RELEASE) && cmake --build --preset=$(PRESET_RELEASE)

clean: clean-debug clean-release

clean-debug:
	cmake --build build/arm-gcc-debug --target clean 2>/dev/null || true

clean-release:
	cmake --build build/arm-gcc-release --target clean 2>/dev/null || true

rebuild: clean-debug
	$(MAKE) debug

rebuild-release: clean-release
	$(MAKE) release

distclean:
	rm -rf build
	@echo "Removed build/"

help:
	@echo "Hylo build (from repo root)"
	@echo ""
	@echo "Firmware:"
	@echo "  make [all]       Build (debug)"
	@echo "  make debug       Same as make"
	@echo "  make release    Build release"
	@echo "  make clean      Clean debug and release build dirs"
	@echo "  make rebuild    Clean debug then build"
	@echo "  make distclean  Remove build/ entirely"
	@echo "  make flash      Build debug + flash via OpenOCD (ST-Link)"
	@echo "  make check-deps Verify cmake, ninja, arm-none-eabi-gcc"
	@echo ""
	@echo "Host tools (Python; scripts use COM10 by default):"
	@echo "  make host-help   List host targets"
	@echo "  make host-monitor        Serial monitor"
	@echo "  make host-enable-streaming  Enable IMU streaming on device"
	@echo "  make host-visualization  IMU 3D visualization"
	@echo ""
	@echo "  make help        This message"

# Host tools (run from repo root; require Python + pyserial, optional: pygame, PyOpenGL)
host host-help:
	@echo "Host targets: host-monitor, host-enable-streaming, host-visualization"
	@echo "Run from repo root. Edit scripts in host/serial and host/visualization to change COM port."

host-monitor:
	cd host/serial && python monitor_serial.py

host-enable-streaming:
	cd host/serial && python enable_streaming.py

host-visualization:
	cd host/visualization && python attitude.py

check-deps:
	@bash scripts/check_deps.sh

flash:
	@bash scripts/flash.sh
