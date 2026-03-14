# Root Makefile: build from repo root (flattened layout).
# Usage: make [target]. Run "make help" for targets.

CONFIG ?= debug
PRESET_DEBUG   := arm-gcc-debug
PRESET_RELEASE := arm-gcc-release

.PHONY: all build clean rebuild help check-deps flash distclean debug release
.PHONY: clean-debug clean-release rebuild-release
.PHONY: host host-help host-monitor host-enable-streaming host-visualization host-interactive
.PHONY: protocol-codegen

all build debug:
	@if [ -z "$(REV)" ]; then echo "Error: REV must be specified (0 or 1). Use: make build REV=0 or make build REV=1"; exit 1; fi
	@if [ "$(REV)" != "0" ] && [ "$(REV)" != "1" ]; then echo "Error: REV must be 0 or 1 (got: $(REV))"; exit 1; fi
	cmake --preset=$(PRESET_DEBUG) -DHW_REV=$(REV) && cmake --build --preset=$(PRESET_DEBUG)

release:
	@if [ -z "$(REV)" ]; then echo "Error: REV must be specified (0 or 1). Use: make release REV=0 or make release REV=1"; exit 1; fi
	@if [ "$(REV)" != "0" ] && [ "$(REV)" != "1" ]; then echo "Error: REV must be 0 or 1 (got: $(REV))"; exit 1; fi
	cmake --preset=$(PRESET_RELEASE) -DHW_REV=$(REV) && cmake --build --preset=$(PRESET_RELEASE)

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

# Regenerate protocol .pb and codegen; verify outputs (requires third_party/nanopb submodule)
protocol-codegen:
	python protocol/verify_codegen.py

help:
	@echo "Hylo build (from repo root)"
	@echo ""
	@echo "Protocol (UART .proto codegen):"
	@echo "  make protocol-codegen  Regenerate .pb.c/.pb.h and protocol_ids/dispatch/tx; verify"
	@echo ""
	@echo "Firmware:"
	@echo "  make build REV=0 or make build REV=1  Build (REV required)"
	@echo "  make release REV=0 or make release REV=1  Release build (REV required)"
	@echo "  make clean       Clean debug and release build dirs"
	@echo "  make rebuild    Clean debug then build"
	@echo "  make distclean  Remove build/ entirely"
	@echo "  make flash REV=0 or make flash REV=1  Flash via OpenOCD (ST-Link); builds first if no build"
	@echo "  make check-deps Verify cmake, ninja, arm-none-eabi-gcc"
	@echo ""
	@echo "Host tools (Python; in tools/host; scripts use COM10 by default):"
	@echo "  make host-help   List host targets"
	@echo "  make host-interactive    UART protocol: listen + send commands (PORT=COM10)"
	@echo "  make host-monitor        Serial monitor"
	@echo "  make host-enable-streaming  Enable IMU streaming on device"
	@echo "  make host-visualization  IMU 3D visualization"
	@echo ""
	@echo "  make help        This message"

# Host tools (run from repo root; require Python + pyserial, optional: pygame, PyOpenGL)
host host-help:
	@echo "Host targets: host-interactive, host-monitor, host-enable-streaming, host-visualization"
	@echo "Run from repo root. For host-interactive: make host-interactive PORT=COM10"

PORT ?= COM10
host-interactive:
	cd tools/host/serial && python uart_protocol_tool.py --port $(PORT) interactive

host-monitor:
	cd tools/host/serial && python monitor_serial.py

host-enable-streaming:
	cd tools/host/serial && python enable_streaming.py

host-visualization:
	cd tools/host/visualization && python attitude.py

check-deps:
	@bash tools/scripts/check_deps.sh

flash:
	@if [ -z "$(REV)" ]; then echo "Error: REV must be specified (0 or 1). Use: make flash REV=0 or make flash REV=1"; exit 1; fi
	@if [ "$(REV)" != "0" ] && [ "$(REV)" != "1" ]; then echo "Error: REV must be 0 or 1 (got: $(REV))"; exit 1; fi
	@bash tools/scripts/flash.sh $(REV)
