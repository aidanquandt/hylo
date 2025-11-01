cmake_minimum_required(VERSION 3.22)
project(firmware C ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(DWT_DW3000 ON  CACHE BOOL "Build DW3000 driver")
set(DWT_DW3720 OFF CACHE BOOL "Build DW3720 driver")

add_compile_options(-ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)

# ---- Subdirs in dependency order (low → high) ----
add_subdirectory(cubemx)                 # produces the 'cubemx' executable/elf
add_subdirectory(platform)               # HAL wrappers (lib)
add_subdirectory(third_party/dwt_uwb_driver)   # vendor UWB driver -> 'uwb_driver'
add_subdirectory(ports)                  # port layers -> 'ports' (includes dwm3000_port)
add_subdirectory(components)             # app logic (lib)

# Link order matters: higher-level depends on lower-level
target_link_libraries(cubemx
    components        # app
    ports             # port layers (dwm3000_port, etc.)
    uwb_driver        # vendor driver (pulls qmath, dw3000, etc.)
    platform          # HAL wrappers
)

# ---- Linker script ----
target_link_options(cubemx PRIVATE
    -T${CMAKE_SOURCE_DIR}/cubemx/STM32H723XG_FLASH.ld
)
