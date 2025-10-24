cmake_minimum_required(VERSION 3.22)

project(firmware C ASM)

# Export compile commands for tooling (clangd, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Reusable options
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Your code libraries
add_subdirectory(app)
add_subdirectory(platform)
add_subdirectory(modules)

# CubeMX-generated executable target lives here
add_subdirectory(cubemx)

# Link your libraries into the CubeMX executable
target_link_libraries(cubemx app platform modules)

# Set linker script path
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/cubemx/STM32H723XG_FLASH.ld")

# Remove any previous or duplicate linker script settings!
target_link_options(cubemx PRIVATE -T${LINKER_SCRIPT})
