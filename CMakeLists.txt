cmake_minimum_required(VERSION 3.22)

# HW revision: 0 = Nucleo (stm32_hwrev_0), 1 = custom PCB (stm32_hwrev_1). Pass -DHW_REV=1 for rev1.
set(HW_REV "0" CACHE STRING "HW revision: 0 or 1")
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/hwconfig/stm32_hwrev_${HW_REV}/cmake/gcc-arm-none-eabi.cmake")

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
if(HW_REV STREQUAL "1")
    add_subdirectory(hwconfig/stm32_hwrev_1)
    set(HWCONFIG_TARGET stm32_hwrev_1)
else()
    add_subdirectory(hwconfig/stm32_hwrev_0)
    set(HWCONFIG_TARGET stm32_hwrev_0)
endif()

# Preprocessor flag for firmware: #if (HWREV == 0) or #if (HWREV == 1)
add_compile_definitions(HWREV=${HW_REV})

add_subdirectory(lib)                       # utility libraries (counter, etc.)
add_subdirectory(protocol)                  # UART protocol (.proto, nanopb, codegen)
add_subdirectory(third_party/dwt_uwb_driver) # vendor UWB -> 'dwt_uwb_driver' (before drivers)
add_subdirectory(third_party/bmi323)        # vendor IMU sensor API -> 'bmi323_api'
add_subdirectory(drivers)   # driver layer (sole interface to HAL/third_party)
add_subdirectory(src)       # app logic

# ---- Layering: firmware links app + drivers + lib only ----
# Firmware target ${HWCONFIG_TARGET} must link only: app, drivers, lib.
# App must not link HAL (stm32cubemx) or third_party (dwt_uwb_driver, bmi323_api) directly;
# all hardware and vendor access goes through the drivers target.
target_link_libraries(${HWCONFIG_TARGET}
    app         # application layer (src/)
    drivers     # driver layer (sole interface to HAL/third_party)
    lib         # utility libraries
)

# ---- Linker script ----
# Force strong RX handlers (protocol_handlers.c) to be linked; otherwise the weak stubs
# in protocol_dispatch.c satisfy refs and the real handlers are dropped from the image.
target_link_options(${HWCONFIG_TARGET} PRIVATE
    -T${CMAKE_SOURCE_DIR}/hwconfig/${HWCONFIG_TARGET}/STM32H723XG_FLASH.ld
    "LINKER:-u,protocol_rx_PingRequest"
    "LINKER:-u,protocol_rx_GetConfigRequest"
    "LINKER:-u,protocol_rx_SetAddressRequest"
)
