# Makefile for building kalman_core as a shared library for offline analysis
#
# Usage:
#   make            - build shared library
#   make clean      - remove build artifacts
#
# The library can be loaded via Python ctypes to reproduce the EKF offline.

CC      := gcc
CFLAGS  := -shared -fPIC -O2 -Wall -Wno-unused-function

# Source from the firmware
KC_SRC  := ../src/sensor_fusion/kalman_core.c

# Use our shims instead of firmware headers
# -I. picks up arm_math_shim.h (as "arm_math.h") and analysis_common.h (as "common.h")
# -I../src/sensor_fusion picks up sf_math.h and kalman_core.h
INCLUDES := -I. -I../src/sensor_fusion

# No extra defines needed — our arm_math.h and common.h shims in this
# directory are found first via -I. and redirect to the real shim headers.
DEFINES :=

# Detect OS for shared library extension
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    LIBEXT  := dylib
    LDFLAGS := -dynamiclib
else
    LIBEXT  := so
    LDFLAGS :=
endif

TARGET := libkalman.$(LIBEXT)

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(KC_SRC) arm_math_shim.h analysis_common.h
	$(CC) $(CFLAGS) $(LDFLAGS) $(INCLUDES) $(DEFINES) -lm -o $@ $(KC_SRC)

clean:
	rm -f $(TARGET)
