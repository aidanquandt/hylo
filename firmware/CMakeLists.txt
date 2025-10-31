cmake_minimum_required(VERSION 3.22)
project(firmware C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(components)
add_subdirectory(cubemx)

# Link all user code to the CubeMX exe in one go
target_link_libraries(cubemx components)

# Linker script
target_link_options(cubemx PRIVATE
    -T${CMAKE_SOURCE_DIR}/cubemx/STM32H723XG_FLASH.ld
)