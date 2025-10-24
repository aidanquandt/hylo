# cmake ..
# cmake -G "MinGW Makefiles" ..

cmake_minimum_required(VERSION 3.10)
project(tdma_sim C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

# Add source files
set(SOURCES
    main.c
    node/node.c
    radio/radio.c
    tdma/tdma.c
    node/tag/tag.c
    node/anchor/anchor.c
)

add_executable(tdma_sim ${SOURCES})

# Add header search paths matching current layout
target_include_directories(tdma_sim PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/node
    ${CMAKE_SOURCE_DIR}/node/tag
    ${CMAKE_SOURCE_DIR}/node/anchor
    ${CMAKE_SOURCE_DIR}/radio
    ${CMAKE_SOURCE_DIR}/tdma
)

target_link_libraries(tdma_sim m)
