# Umbrella libraries (target name "components" kept for link compatibility)
add_library(components INTERFACE)
add_library(components_inc INTERFACE)

# Global dependencies - app logic needs platform, ports, drivers, utils, and common
target_link_libraries(components_inc INTERFACE
    common
    stm32cubemx
    platform
    ports
    uwb_driver_itf
    lib
    CMSIS-DSP
)

# Include directories - one per module (flat layout: src/<module>/)
target_include_directories(components_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/module
    ${CMAKE_CURRENT_SOURCE_DIR}/datalogger
    ${CMAKE_CURRENT_SOURCE_DIR}/error_handler
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/ota_config
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/twr
    ${CMAKE_CURRENT_SOURCE_DIR}/twr_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/uart_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb_node
    ${CMAKE_CURRENT_SOURCE_DIR}/watchdog
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi
)

# Auto-generate wifi_config.h from example if not present (file is gitignored)
set(WIFI_CONFIG     "${CMAKE_CURRENT_SOURCE_DIR}/wifi/wifi_config.h")
set(WIFI_CONFIG_EX  "${CMAKE_CURRENT_SOURCE_DIR}/wifi/wifi_config.h.example")
if(NOT EXISTS "${WIFI_CONFIG}")
    message(WARNING
        "\n"
        "  wifi_config.h not found - copying from wifi_config.h.example.\n"
        "  Edit src/wifi/wifi_config.h with your credentials before flashing.\n")
    configure_file("${WIFI_CONFIG_EX}" "${WIFI_CONFIG}" COPYONLY)
endif()

function(add_component_module MODULE_NAME SOURCES)
    add_library(${MODULE_NAME} STATIC ${SOURCES})
    target_link_libraries(${MODULE_NAME} PUBLIC components_inc)
    set_target_properties(${MODULE_NAME} PROPERTIES FOLDER "src")
    target_link_libraries(components INTERFACE ${MODULE_NAME})
endfunction()

set(APP_SOURCES
    app/app.c
)
add_component_module(app "${APP_SOURCES}")

set(MODULE_SOURCES
    module/module.c
    datalogger/datalogger.c
    error_handler/error_handler.c
    imu/imu.c
    ota_config/ota_config.c
    sensor_fusion/kalman_core.c
    sensor_fusion/sensor_fusion.c
    twr/twr.c
    twr/twr_algorithm.c
    twr/twr_state_machine.c
    twr/initiator.c
    twr/responder.c
    twr_manager/twr_manager.c
    twr_manager/twr_scheduler.c
    uart_manager/uart_manager.c
    uart_manager/uart_cmd_router.c
    uwb/uwb.c
    uwb_node/uwb_node.c
    watchdog/watchdog.c
    wifi/wifi.c
    wifi/wifi_ota_parser.c
)
add_component_module(modules "${MODULE_SOURCES}")
