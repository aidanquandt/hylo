file(GLOB UWB_PORT_SRC src/*.c)
add_library(uwb_qorvo_port STATIC ${UWB_PORT_SRC})

target_include_directories(uwb_qorvo_port PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link to vendor driver and your platform layer
target_link_libraries(uwb_qorvo_port PUBLIC uwb_driver platform)
