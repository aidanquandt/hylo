# Utility libraries
add_library(lib STATIC)

target_sources(lib
    PRIVATE
        counter/counter.c
        state_machine/state_machine.c
        stopwatch/stopwatch.c
)

target_include_directories(lib
    PUBLIC
        common
        counter
        state_machine
        stopwatch
        mac_802154
        uwb_protocol
    PRIVATE
        ${PROJECT_SOURCE_DIR}/platform/timer
)

target_link_libraries(lib PRIVATE timer)
