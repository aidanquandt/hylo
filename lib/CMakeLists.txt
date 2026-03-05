# Utility libraries - reusable components with minimal dependencies

# Explicit source list for maintainability
set(LIB_SOURCES
    backoff/backoff.c
    counter/counter.c
    state_machine/state_machine.c
    stopwatch/stopwatch.c
)

add_library(lib STATIC ${LIB_SOURCES})

# Public headers (what users of 'lib' can include)
target_include_directories(lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/backoff
        ${CMAKE_CURRENT_SOURCE_DIR}/counter
        ${CMAKE_CURRENT_SOURCE_DIR}/state_machine
        ${CMAKE_CURRENT_SOURCE_DIR}/stopwatch
        ${CMAKE_CURRENT_SOURCE_DIR}/mac_802154
        ${CMAKE_CURRENT_SOURCE_DIR}/uwb_protocol
)

# Dependencies - lib uses common types and timer from platform
target_link_libraries(lib 
    PUBLIC common    # Need common.h types/macros
    PRIVATE timer    # Internal use of platform timer
)
