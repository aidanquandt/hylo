# Umbrella libraries
add_library(platform INTERFACE)      # what top-level links to
add_library(platform_inc INTERFACE)  # global include + usage

# Global usage (one place)
target_link_libraries(platform_inc INTERFACE stm32cubemx components)
target_include_directories(platform_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

function(add_component NAME)
    set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/${NAME}")
    if(NOT EXISTS "${ROOT}")
        return()
    endif()

    set(INCS "${ROOT}")
    file(GLOB_RECURSE _dirs LIST_DIRECTORIES TRUE "${ROOT}/*")
    foreach(d ${_dirs})
        if(IS_DIRECTORY "${d}")
            list(APPEND INCS "${d}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES INCS)

    target_include_directories(platform_inc INTERFACE ${INCS})

    file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
        "${ROOT}/*.c" "${ROOT}/*.cpp" "${ROOT}/*.cxx"
    )
    if(SRCS)
        add_library(${NAME} STATIC ${SRCS})
        target_link_libraries(${NAME} PUBLIC platform_inc)
        set_target_properties(${NAME} PROPERTIES FOLDER "platform")
        target_link_libraries(platform INTERFACE ${NAME})
    endif()
endfunction()

file(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*")
foreach(child ${children})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${child}")
        add_component(${child})
    endif()
endforeach()