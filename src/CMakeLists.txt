# App layer umbrella: app = all app libs; components_inc = include + link interface for all app modules.
add_library(app INTERFACE)
add_library(components_inc INTERFACE)

# system_halt: minimal lib for fatal halt (sends SystemFatalEvent then halts)
add_library(system_halt STATIC system_halt/system_halt.c)
target_include_directories(system_halt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(system_halt PUBLIC protocol common drivers)
set_target_properties(system_halt PROPERTIES FOLDER "src")

# App must not link HAL, third_party, or hwconfig; driver includes/deps come via the drivers target only.
target_link_libraries(components_inc INTERFACE
    common
    drivers
    lib
    protocol
    system_halt
    CMSIS-DSP
)

# Include directories - one per module (flat layout: src/<module>/)
target_include_directories(components_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/module
    ${CMAKE_CURRENT_SOURCE_DIR}/datalogger
    ${CMAKE_CURRENT_SOURCE_DIR}/imu
    ${CMAKE_CURRENT_SOURCE_DIR}/ota_config
    ${CMAKE_CURRENT_SOURCE_DIR}/sensor_fusion
    ${CMAKE_CURRENT_SOURCE_DIR}/twr
    ${CMAKE_CURRENT_SOURCE_DIR}/twr_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb_node
    ${CMAKE_CURRENT_SOURCE_DIR}/uart_protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/system_halt
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
    target_link_libraries(app INTERFACE ${MODULE_NAME})
endfunction()

set(APP_SOURCES
    app/app.c
)
add_component_module(app_main "${APP_SOURCES}")

# UART protocol: framing (COBS+CRC16) and RX task; links protocol + drivers
set(UART_PROTOCOL_SOURCES
    uart_protocol/uart_framing.c
    uart_protocol/uart_protocol_task.c
    uart_protocol/protocol_handlers.c
)
add_component_module(uart_protocol "${UART_PROTOCOL_SOURCES}")

set(MODULE_SOURCES
    module/module.c
    datalogger/datalogger.c
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
    uwb/uwb.c
    uwb_node/uwb_node.c
    watchdog/watchdog.c
    wifi/wifi.c
    wifi/wifi_ota_parser.c
)
add_component_module(modules "${MODULE_SOURCES}")
