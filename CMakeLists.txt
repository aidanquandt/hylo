cmake_minimum_required(VERSION 3.22)
project(hylo
    VERSION 1.0.0
    DESCRIPTION "Hylo UWB Firmware"
    LANGUAGES C ASM
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Create INTERFACE library for global common includes
# This is better than include_directories() as it's explicit and trackable
add_library(common INTERFACE)
target_include_directories(common INTERFACE ${CMAKE_SOURCE_DIR}/common)

# Compiler warnings and optimizations
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Werror=implicit-function-declaration
        -Wno-unused-parameter
    )
endif()

set(DWT_DW3000 ON  CACHE BOOL "Build DW3000 driver")
set(DWT_DW3720 OFF CACHE BOOL "Build DW3720 driver")

add_compile_options(-ffunction-sections -fdata-sections)
add_link_options(-Wl,--gc-sections)

# ---- Subdirs in dependency order (low → high) ----
add_subdirectory(config)                 # device/board configurations
add_subdirectory(cubemx)                 # produces the 'cubemx' executable/elf
add_subdirectory(platform)               # HAL wrappers (lib)
add_subdirectory(lib)                    # utility libraries (counter, etc.)
add_subdirectory(third_party/dwt_uwb_driver)   # vendor UWB driver -> 'uwb_driver'
add_subdirectory(third_party/bmi323)     # vendor IMU sensor API -> 'bmi323_api'
add_subdirectory(ports)                  # port layers -> 'ports' (includes uwb_port, imu_port)
add_subdirectory(src)                    # app logic (lib)

# Link order matters: higher-level depends on lower-level
target_link_libraries(cubemx
    components        # app
    ports             # port layers (uwb_port, imu_port)
    uwb_driver        # vendor UWB driver (DW3000-specific implementation)
    bmi323_api        # vendor IMU sensor API (BMI323-specific implementation)
    platform          # HAL wrappers
    lib               # utility libraries
)

# ---- Linker script ----
target_link_options(cubemx PRIVATE
    -T${CMAKE_SOURCE_DIR}/cubemx/STM32H723XG_FLASH.ld
)
