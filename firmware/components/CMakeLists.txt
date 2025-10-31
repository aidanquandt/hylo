# Umbrella libraries
add_library(components INTERFACE)      # what top-level links to
add_library(components_inc INTERFACE)  # global include + usage

# Global usage (one place)
target_link_libraries(components_inc INTERFACE stm32cubemx)   # + m if needed
target_include_directories(components_inc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}  # lets you do #include "modules/tdma/tdma.h" if you want
)

function(add_component NAME)
    set(ROOT "${CMAKE_CURRENT_SOURCE_DIR}/${NAME}")
    if(NOT EXISTS "${ROOT}")
        return()
    endif()

    # Collect include dirs: this folder + all subfolders (so bare headers work)
    set(INCS "${ROOT}")
    file(GLOB_RECURSE _dirs LIST_DIRECTORIES TRUE "${ROOT}/*")
    foreach(d ${_dirs})
        if(IS_DIRECTORY "${d}")
            list(APPEND INCS "${d}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES INCS)

    # Every component’s include dirs go into the global pool
    target_include_directories(components_inc INTERFACE ${INCS})

    # Sources for this component
    file(GLOB_RECURSE SRCS CONFIGURE_DEPENDS
        "${ROOT}/*.c" "${ROOT}/*.cpp" "${ROOT}/*.cxx"
    )
    if(SRCS)
        add_library(${NAME} STATIC ${SRCS})
        target_link_libraries(${NAME} PUBLIC components_inc)
        set_target_properties(${NAME} PROPERTIES FOLDER "components")
        target_link_libraries(components INTERFACE ${NAME})
    endif()
endfunction()

file(GLOB children RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*")
foreach(child ${children})
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${child}")
        add_component(${child})
    endif()
endforeach()