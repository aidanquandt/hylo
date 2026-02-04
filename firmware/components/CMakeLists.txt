# Umbrella libraries
add_library(components INTERFACE)      # what top-level links to
add_library(components_inc INTERFACE)  # global include + usage

# Global dependencies - components need platform, ports, drivers, utils, and common
target_link_libraries(components_inc INTERFACE 
    common            # Global common types/macros
    stm32cubemx       # HAL
    platform          # Platform HAL wrappers
    ports             # Hardware port adapters
    uwb_driver_itf    # UWB driver interface
    lib               # Utility libraries
)

# Include directories - only top-level and module roots
# Individual module subdirs are accessed via relative paths in #include
target_include_directories(components_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/datalogger
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/error_handler
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/ota_config
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/sensor_fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/twr
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/twr/twr_engine
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/twr/twr_roles
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/twr_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/twr_manager/twr_scheduler
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/uart_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/uwb
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/uwb_node
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/watchdog
)

# Helper function to add component modules with consistent structure
function(add_component_module MODULE_NAME SOURCES)
    add_library(${MODULE_NAME} STATIC ${SOURCES})
    target_link_libraries(${MODULE_NAME} PUBLIC components_inc)
    set_target_properties(${MODULE_NAME} PROPERTIES FOLDER "components")
    target_link_libraries(components INTERFACE ${MODULE_NAME})
endfunction()

# App library - explicit source listing (update when adding files)
set(APP_SOURCES
    app/app.c
)
add_component_module(app "${APP_SOURCES}")

# Modules library - explicit source listing (update when adding files)
set(MODULE_SOURCES
    modules/module.c
    modules/datalogger/datalogger.c
    modules/error_handler/error_handler.c
    modules/imu/imu.c
    modules/ota_config/ota_config.c
    modules/sensor_fusion/sensor_fusion.c
    modules/twr/twr.c
    modules/twr/twr_engine/twr_algorithm.c
    modules/twr/twr_engine/twr_state_machine.c
    modules/twr/twr_roles/initiator/initiator.c
    modules/twr/twr_roles/responder/responder.c
    modules/twr_manager/twr_manager.c
    modules/twr_manager/twr_scheduler/twr_scheduler.c
    modules/uart_manager/uart_manager.c
    modules/uart_manager/uart_cmd_router.c
    modules/uwb/uwb.c
    modules/uwb_node/uwb_node.c
    modules/watchdog/watchdog.c
)
add_component_module(modules "${MODULE_SOURCES}")