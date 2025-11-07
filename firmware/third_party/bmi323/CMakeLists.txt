# BMI323/BMI330 SensorAPI library
cmake_minimum_required(VERSION 3.13.1)

project(bmi323_api)

# Interface library for include directories
add_library(bmi323_itf INTERFACE)
target_include_directories(bmi323_itf INTERFACE .)

# Main BMI323 API library
add_library(bmi323_api STATIC
    bmi3.c
    bmi323.c
    bmi330.c
)

target_link_libraries(bmi323_api 
    PUBLIC bmi323_itf
)

# Compiler settings for vendor code
target_compile_options(bmi323_api PRIVATE
    -Wno-unused-parameter
    -Wno-unused-variable
)
