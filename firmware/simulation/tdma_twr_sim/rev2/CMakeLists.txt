cmake_minimum_required(VERSION 3.16)
project(tdma_twr_sim C)

# ------------------------------------------------------------------------------
# How to configure, build, run, and open GTKWave
#
# Single-config generators (Ninja/Makefiles):
#   Configure (Release):
#     cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
#   Build:
#     cmake --build build
#   Run simulator (writes trace.vcd in runtime dir):
#     cmake --build build --target run
#   Open trace in GTKWave:
#     cmake --build build --target wave
#
# Multi-config generators (Visual Studio, Ninja Multi-Config):
#   Configure:
#     cmake -S . -B build -G "Visual Studio 17 2022" -A x64
#     # or: cmake -S . -B build -G "Ninja Multi-Config"
#   Build (Release):
#     cmake --build build --config Release
#   Run / GTKWave (Release):
#     cmake --build build --config Release --target run
#     cmake --build build --config Release --target wave
#
# Output location option:
#   Place sim next to sources:
#     cmake -S . -B build -DSIM_OUTPUT_IN_SOURCE=ON
#   Default (OFF) places sim in the build directory.
#
# GTKWave:
#   Ensure "gtkwave" is in PATH or pass an explicit path:
#     cmake -S . -B build -DGTKWAVE_EXECUTABLE="C:/Path/To/gtkwave.exe"
#
# Notes:
#   - After build, CMake prints the resolved sim path.
#   - trace.vcd is written to the runtime directory printed/configured below.
# ------------------------------------------------------------------------------

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Sources (adjust if your file set differs)
set(SIM_SRC
    main.c
    sim/simulation.c
    core/node.c
    sim/phy.c
    sim/timebase.c
    sim/topology.c
    trace/trace_vcd.c
)

add_executable(sim ${SIM_SRC})

target_include_directories(sim PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Warnings
if(MSVC)
    target_compile_options(sim PRIVATE /W4 /permissive- /D_CRT_SECURE_NO_WARNINGS)
else()
    target_compile_options(sim PRIVATE -Wall -Wextra -Wpedantic)
    # Link libm for sqrtf
    target_link_libraries(sim PRIVATE m)
endif()

# Where to place sim.exe:
# OFF (default): in this subdir's build folder (…/tdma_twr_sim/build/sim.exe)
# ON:            next to sources (…/tdma_twr_sim/sim.exe)
option(SIM_OUTPUT_IN_SOURCE "Place sim.exe next to sources" OFF)

if(SIM_OUTPUT_IN_SOURCE)
    set(_RUNTIME_DIR "${CMAKE_CURRENT_LIST_DIR}")
else()
    set(_RUNTIME_DIR "${CMAKE_CURRENT_BINARY_DIR}")
endif()

set_target_properties(sim PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${_RUNTIME_DIR}"
)

# Multi-config generators (VS): set per-config dirs too
foreach(cfg IN ITEMS Debug Release RelWithDebInfo MinSizeRel)
    set_target_properties(sim PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${cfg} "${_RUNTIME_DIR}"
    )
endforeach()

# After build, print the resolved output path so you can copy/paste it
add_custom_command(TARGET sim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "Built sim at: $<TARGET_FILE:sim>"
)

# Convenience targets: run simulator and open GTKWave
add_custom_target(run
    DEPENDS sim
    COMMAND $<TARGET_FILE:sim>
    WORKING_DIRECTORY "${_RUNTIME_DIR}"
    USES_TERMINAL
    COMMENT "Run simulator (writes trace.vcd in runtime dir)"
)

# GTKWave integration with optional save file (.gtkw/.sav)
# Create your view once in GTKWave, then File -> Write Save File as 'wave.gtkw' (recommended)
# next to this CMakeLists.txt or in the runtime dir. Or pass a custom path:
#   -DGTKWAVE_SAVE_FILE="C:/path/to/view.gtkw"
set(GTKWAVE_SAVE_FILE "" CACHE STRING "Path to a GTKWave save file (.gtkw or .sav) to preload")

find_program(GTKWAVE_EXECUTABLE NAMES gtkwave gtkwave.exe)
if (GTKWAVE_EXECUTABLE)
    # Pick save file: explicit cache var or common defaults
    set(_GTKWAVE_SAVE "")
    if (GTKWAVE_SAVE_FILE)
        set(_GTKWAVE_SAVE "${GTKWAVE_SAVE_FILE}")
    else()
        set(_GTKWAVE_SAVE_CANDIDATES
            "${CMAKE_CURRENT_LIST_DIR}/wave.gtkw"
            "${CMAKE_CURRENT_LIST_DIR}/wave.sav"
            "${_RUNTIME_DIR}/wave.gtkw"
            "${_RUNTIME_DIR}/wave.sav"
        )
        foreach(_f IN LISTS _GTKWAVE_SAVE_CANDIDATES)
            if (EXISTS "${_f}")
                set(_GTKWAVE_SAVE "${_f}")
                break()
            endif()
        endforeach()
    endif()

    # Build arguments safely as a list (first: trace.vcd, optional second: save file)
    set(_GTKWAVE_ARGS "${_RUNTIME_DIR}/trace.vcd")
    if (_GTKWAVE_SAVE)
        list(APPEND _GTKWAVE_ARGS "${_GTKWAVE_SAVE}")
    endif()

    # wave_only: open the last trace without re-running the simulator
    add_custom_target(wave_only
        COMMAND ${CMAKE_COMMAND} -E echo "Opening GTKWave (no run) with$<IF:$<BOOL:${_GTKWAVE_SAVE}>, save file: ${_GTKWAVE_SAVE},out save file>"
        COMMAND "${GTKWAVE_EXECUTABLE}" ${_GTKWAVE_ARGS}
        WORKING_DIRECTORY "${_RUNTIME_DIR}"
        USES_TERMINAL
        COMMENT "Open existing trace.vcd in GTKWave"
    )

    # wave_blocking: rebuild, run, then open GTKWave in the foreground (legacy behavior)
    add_custom_target(wave_blocking
        DEPENDS run
        COMMAND ${CMAKE_COMMAND} -E echo "Opening GTKWave (blocking) with$<IF:$<BOOL:${_GTKWAVE_SAVE}>, save file: ${_GTKWAVE_SAVE},out save file>"
        COMMAND "${GTKWAVE_EXECUTABLE}" ${_GTKWAVE_ARGS}
        WORKING_DIRECTORY "${_RUNTIME_DIR}"
        USES_TERMINAL
        COMMENT "Open trace.vcd in GTKWave (blocking)"
    )

    # wave (default): rebuild, run, then detach GTKWave so the build finishes cleanly
    if (WIN32)
        add_custom_target(wave
            DEPENDS run
            COMMAND ${CMAKE_COMMAND} -E echo "Opening GTKWave (detached) with$<IF:$<BOOL:${_GTKWAVE_SAVE}>, save file: ${_GTKWAVE_SAVE},out save file>"
            COMMAND cmd.exe /C start "" "${GTKWAVE_EXECUTABLE}" ${_GTKWAVE_ARGS}
            WORKING_DIRECTORY "${_RUNTIME_DIR}"
            USES_TERMINAL
            COMMENT "Open trace.vcd in GTKWave (detached)"
        )
    else()
        # On non-Windows, fall back to blocking (detaching is shell-specific)
        add_custom_target(wave DEPENDS wave_blocking)
    endif()
else()
    add_custom_target(wwave
        DEPENDS run
        COMMAND ${CMAKE_COMMAND} -E echo "GTKWave not found. Add it to PATH or set GTKWAVE_EXECUTABLE."
    )
endif()

