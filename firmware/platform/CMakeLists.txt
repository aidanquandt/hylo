add_library(platform STATIC
    src/platform.c
)

target_include_directories(platform
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Platform uses HAL types optionally
target_link_libraries(platform
    PUBLIC
        stm32cubemx
)
