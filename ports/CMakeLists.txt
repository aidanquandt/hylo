# Port layers - adapters between vendor drivers and platform HAL

add_library(ports INTERFACE)

# Ports expose their own headers
target_include_directories(ports INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/imu_port
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb_port
)

# Helper function to add port modules with consistent structure
function(add_port_module PORT_NAME)
    add_library(${PORT_NAME} STATIC ${PORT_NAME}/${PORT_NAME}.c)
    set_target_properties(${PORT_NAME} PROPERTIES FOLDER "ports")
    target_link_libraries(ports INTERFACE ${PORT_NAME})
    # Additional dependencies passed as ARGN
    if(ARGN)
        target_link_libraries(${PORT_NAME} PUBLIC ${ARGN})
    endif()
endfunction()

# IMU Port - links platform, vendor driver, and common
add_port_module(imu_port 
    platform      # Platform HAL
    bmi323_api    # BMI323 vendor driver
    common        # Global types/macros
)

# UWB Port - links platform, vendor driver, and common
add_port_module(uwb_port 
    platform       # Platform HAL
    uwb_driver_itf # UWB vendor driver interface
    common         # Global types/macros
    lib            # For stopwatch utility
)


