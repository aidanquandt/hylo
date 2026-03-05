# Board/device-specific configuration

add_library(config STATIC
    device_mapping.c
)

target_include_directories(config
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Config only needs basic types from common
target_link_libraries(config
    PUBLIC common
)
