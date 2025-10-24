add_library(platform STATIC
    platform.c
)

# Expose include path so consumers can include
target_include_directories(platform
    PUBLIC
        ${CMAKE_SOURCE_DIR}
)

# App may use HAL/BSP headers via stm32cubemx interface
# (provided by cubemx/cmake/stm32cubemx)
target_link_libraries(platform
    PUBLIC
        stm32cubemx
)
