# Root Makefile: build from repo root (flattened layout).
# Usage: make [target]. Run "make help" for targets.

CONFIG ?= debug
PRESET_DEBUG := arm-gcc-debug

.PHONY: all build clean rebuild help check-deps flash distclean debug
.PHONY: clean-debug
.PHONY: host host-help host-monitor host-enable-streaming host-visualization host-interactive host-webapp
.PHONY: protocol-codegen ensure-protocol-codegen

# Regenerate protocol if generated files are missing or .proto files are newer
ensure-protocol-codegen:
	@if [ ! -f generated/protocol/c/protocol.pb.c ] || [ ! -f generated/protocol/c/host_options.pb.c ] || \
	     [ protocol/protocol.proto -nt generated/protocol/c/protocol.pb.c ] || \
	     [ protocol/host_options.proto -nt generated/protocol/c/host_options.pb.c ]; then \
		echo "Protocol codegen missing or out of date, regenerating..."; \
		$(MAKE) protocol-codegen; \
	fi

all build debug: ensure-protocol-codegen
	@if [ -z "$(REV)" ]; then echo "Error: REV must be specified (0 or 1). Use: make build REV=0 or make build REV=1"; exit 1; fi
	@if [ "$(REV)" != "0" ] && [ "$(REV)" != "1" ]; then echo "Error: REV must be 0 or 1 (got: $(REV))"; exit 1; fi
	cmake --preset=$(PRESET_DEBUG) -DHW_REV=$(REV) && cmake --build --preset=$(PRESET_DEBUG)

clean: clean-debug

clean-debug:
	cmake --build build/arm-gcc-debug --target clean 2>/dev/null || true

rebuild: clean-debug
	$(MAKE) debug

distclean:
	rm -rf build
	@echo "Removed build/"

# Regenerate protocol .pb and codegen; verify outputs (requires third_party/nanopb submodule)
protocol-codegen:
	python tools/protocol_codegen/verify_codegen.py

help:
	@echo "Hylo build (from repo root)"
	@echo ""
	@echo "Protocol (.proto codegen):"
	@echo "  make protocol-codegen  Regenerate .pb.c/.pb.h and protocol_ids/dispatch/tx; verify"
	@echo "  (build runs this automatically if generated files are missing or .proto changed)"
	@echo ""
	@echo "Firmware:"
	@echo "  make build REV=0 or make build REV=1  Build (REV required)"
	@echo "  make clean       Clean build dir"
	@echo "  make rebuild    Clean debug then build"
	@echo "  make distclean  Remove build/ entirely"
	@echo "  make flash REV=0 or make flash REV=1  Flash via OpenOCD (ST-Link); builds first if no build"
	@echo "  make check-deps Verify cmake, ninja, arm-none-eabi-gcc"
	@echo ""
	@echo "Host tools (Python; in host/; scripts use /dev/ttyUSB0 by default on Linux/WSL):"
	@echo "  make host-help   List host targets"
	@echo "  make host-interactive    Protocol: listen + send commands (PORT=/dev/ttyUSB0)"
	@echo "  make host-monitor        Serial monitor"
	@echo "  make host-enable-streaming  Enable IMU streaming on device"
	@echo "  make host-visualization  IMU 3D visualization"
	@echo "  make host-webapp        Web UI (FastAPI); serves at http://127.0.0.1:8000"
	@echo ""
	@echo "  make help        This message"

# Host tools (run from repo root; require Python + pyserial, optional: pygame, PyOpenGL)
host host-help:
	@echo "Host targets: host-interactive, host-monitor, host-enable-streaming, host-visualization, host-webapp"
	@echo "Run from repo root. For host-interactive: make host-interactive [PORT=/dev/ttyUSB0]"

PORT ?= /dev/ttyUSB0
host-interactive:
	@bash tools/scripts/attach-usb.sh serial
	cd host/serial && python protocol_tool.py --port $(PORT) interactive

host-monitor:
	@bash tools/scripts/attach-usb.sh serial
	cd host/serial && python monitor_serial.py --port $(PORT)

host-enable-streaming:
	@bash tools/scripts/attach-usb.sh serial
	cd host/serial && python enable_streaming.py --port $(PORT)

host-visualization:
	@bash tools/scripts/attach-usb.sh serial
	cd host/visualization && python attitude.py --port $(PORT)

host-webapp:
	@bash tools/scripts/attach-usb.sh serial
	(python -c "import time, webbrowser; time.sleep(2); webbrowser.open('http://127.0.0.1:8000')") &
	uvicorn host.webapp.backend.main:app --reload

check-deps:
	@bash tools/scripts/check_deps.sh

flash:
	@if [ -z "$(REV)" ]; then echo "Error: REV must be specified (0 or 1). Use: make flash REV=0 or make flash REV=1"; exit 1; fi
	@if [ "$(REV)" != "0" ] && [ "$(REV)" != "1" ]; then echo "Error: REV must be 0 or 1 (got: $(REV))"; exit 1; fi
	@bash tools/scripts/flash.sh $(REV)
