# Port layers - adapters between vendor drivers and platform HAL

add_library(ports INTERFACE)

function(add_port NAME)
    set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/${NAME}")
    if(NOT EXISTS "${ROOT}")
        return()
    endif()

    # Add include directories
    set(INCS "${ROOT}")
    file(GLOB_RECURSE _dirs LIST_DIRECTORIES TRUE "${ROOT}/*")
    foreach(d ${_dirs})
        if(IS_DIRECTORY "${d}")
            list(APPEND INCS "${d}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES INCS)

    target_include_directories(ports INTERFACE ${INCS})

    # Add source files
    file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
        "${ROOT}/*.c" "${ROOT}/*.cpp" "${ROOT}/*.cxx"
    )
    if(SRCS)
        add_library(${NAME}_port STATIC ${SRCS})
        # Link to platform and appropriate vendor driver interface
        if(NAME STREQUAL "dw3000")
            target_link_libraries(${NAME}_port PUBLIC platform uwb_driver_itf)
        elseif(NAME STREQUAL "bmi323")
            target_link_libraries(${NAME}_port PUBLIC platform bmi323_itf)
        else()
            target_link_libraries(${NAME}_port PUBLIC platform)
        endif()
        set_target_properties(${NAME}_port PROPERTIES FOLDER "ports")
        target_link_libraries(ports INTERFACE ${NAME}_port)
    endif()
endfunction()

# Add each port
file(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*")
foreach(child ${children})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${child}")
        add_port(${child})
    endif()
endforeach()

