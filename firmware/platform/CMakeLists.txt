# Umbrella libraries
add_library(platform INTERFACE)      # what top-level links to
add_library(platform_inc INTERFACE)  # global include + usage

# Platform dependencies - HAL and common types
target_link_libraries(platform_inc INTERFACE 
    stm32cubemx    # STM32 HAL
    common         # Global common types/macros
)

# Explicit include directories for each platform module
target_include_directories(platform_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/gpio
    ${CMAKE_CURRENT_SOURCE_DIR}/os
    ${CMAKE_CURRENT_SOURCE_DIR}/spi
    ${CMAKE_CURRENT_SOURCE_DIR}/system
    ${CMAKE_CURRENT_SOURCE_DIR}/timer
    ${CMAKE_CURRENT_SOURCE_DIR}/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog
)

# Helper function to add platform modules with consistent structure
function(add_platform_module MODULE_NAME)
    add_library(${MODULE_NAME} STATIC ${MODULE_NAME}/platform_${MODULE_NAME}.c)
    target_link_libraries(${MODULE_NAME} PUBLIC platform_inc)
    set_target_properties(${MODULE_NAME} PROPERTIES FOLDER "platform")
    target_link_libraries(platform INTERFACE ${MODULE_NAME})
endfunction()

# Platform modules
add_platform_module(gpio)
add_platform_module(os)
add_platform_module(spi)
add_platform_module(timer)
add_platform_module(uart)
add_platform_module(watchdog)

# System module has special dependency on config for device mapping
add_library(system STATIC system/platform_system.c)
target_link_libraries(system PUBLIC platform_inc config)
set_target_properties(system PROPERTIES FOLDER "platform")
target_link_libraries(platform INTERFACE system)