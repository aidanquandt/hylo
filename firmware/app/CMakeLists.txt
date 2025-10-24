add_library(app STATIC
    src/app.c
)

# Public headers for consumers (e.g., cubemx executable)
target_include_directories(app
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# App may use HAL/BSP headers via stm32cubemx interface
# (provided by cubemx/cmake/stm32cubemx)
target_link_libraries(app
    PUBLIC
        stm32cubemx
)
