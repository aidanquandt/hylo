# Port layers - adapters between vendor drivers and platform HAL

add_library(ports INTERFACE)

# Ports expose their own headers
target_include_directories(ports INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/imu_port
    ${CMAKE_CURRENT_SOURCE_DIR}/uwb_port
)

# IMU Port - links platform, vendor driver, and common
add_library(imu_port STATIC
    imu_port/imu_port.c
)
target_link_libraries(imu_port PUBLIC 
    platform      # Platform HAL
    bmi323_api    # BMI323 vendor driver
    common        # Global types/macros
)
set_target_properties(imu_port PROPERTIES FOLDER "ports")
target_link_libraries(ports INTERFACE imu_port)

# UWB Port - links platform, vendor driver, and common
add_library(uwb_port STATIC
    uwb_port/uwb_port.c
)
target_link_libraries(uwb_port PUBLIC 
    platform       # Platform HAL
    uwb_driver_itf # UWB vendor driver interface
    common         # Global types/macros
    lib            # For stopwatch utility
)
set_target_properties(uwb_port PROPERTIES FOLDER "ports")
target_link_libraries(ports INTERFACE uwb_port)

