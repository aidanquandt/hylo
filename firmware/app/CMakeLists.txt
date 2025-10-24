add_library(app STATIC
    app.c
)

# Expose include path so consumers can include "app/app.h"
target_include_directories(app
    PUBLIC
        ${CMAKE_SOURCE_DIR}
)

# App may use HAL/BSP headers via stm32cubemx interface
# (provided by cubemx/cmake/stm32cubemx)
target_link_libraries(app
    PUBLIC
        stm32cubemx
)
