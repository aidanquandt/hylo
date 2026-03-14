# Umbrella driver layer - sole interface between app and HAL/third_party.
# The firmware links only this INTERFACE target (not individual driver libs); app/driver boundary is a single dependency.
add_library(drivers INTERFACE)

# Helper: add a HAL-based driver module (links stm32cubemx, common; optional extra deps e.g. config for system_driver)
function(add_driver_module MODULE_NAME)
    add_library(${MODULE_NAME} STATIC ${MODULE_NAME}/${MODULE_NAME}.c)
    target_include_directories(${MODULE_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_NAME})
    target_link_libraries(${MODULE_NAME} PUBLIC stm32cubemx common ${ARGN})
    set_target_properties(${MODULE_NAME} PROPERTIES FOLDER "drivers")
    target_link_libraries(drivers INTERFACE ${MODULE_NAME})
    target_include_directories(drivers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/${MODULE_NAME})
endfunction()

# HAL-based drivers (order: uart, gpio, spi, timer, os, watchdog, system)
add_driver_module(uart_driver)
add_driver_module(gpio_driver)
add_driver_module(spi_driver)
add_driver_module(timer_driver)
add_driver_module(os_driver)
add_driver_module(watchdog_driver)
add_driver_module(sdcard_driver)
add_driver_module(system_driver config)

# IMU driver: wraps bmi323 + spi/gpio/timer/os drivers
add_library(imu_driver STATIC imu_driver/imu_driver.c)
target_include_directories(imu_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/imu_driver)
target_link_libraries(imu_driver PUBLIC stm32cubemx common bmi323_api spi_driver os_driver timer_driver gpio_driver)
set_target_properties(imu_driver PROPERTIES FOLDER "drivers")
target_link_libraries(drivers INTERFACE imu_driver)
target_include_directories(drivers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/imu_driver)

# UWB driver: wraps DWT vendor driver + spi/gpio/os/timer drivers
add_library(uwb_driver STATIC uwb_driver/uwb_driver.c)
target_include_directories(uwb_driver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/uwb_driver)
target_link_libraries(uwb_driver PUBLIC dwt_uwb_driver uwb_driver_itf spi_driver gpio_driver os_driver timer_driver lib common)
set_target_properties(uwb_driver PROPERTIES FOLDER "drivers")
target_link_libraries(drivers INTERFACE uwb_driver)
target_include_directories(drivers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/uwb_driver)
