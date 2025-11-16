# Utility libraries
add_library(lib STATIC)

target_sources(lib
    PRIVATE
        counter/counter.c
        state_machine/state_machine.c
)

target_include_directories(lib
    PUBLIC
        common
        counter
        state_machine
        mac_802154
)
