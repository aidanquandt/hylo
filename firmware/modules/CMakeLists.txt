# Collect all module sources and include directories
set(MODULES_SOURCES "")
set(MODULES_INCLUDES "")

function(register_module name)
    set(MOD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${name})
    if(NOT IS_DIRECTORY ${MOD_DIR})
        message(FATAL_ERROR "Module '${name}' directory not found: ${MOD_DIR}")
    endif()

    file(GLOB MODULE_SOURCES CONFIGURE_DEPENDS
        ${MOD_DIR}/*.c
        ${MOD_DIR}/*.cpp
        ${MOD_DIR}/*.cxx
    )

    if(MODULE_SOURCES)
        list(APPEND MODULES_SOURCES ${MODULE_SOURCES})
    endif()

    # Add the module directory itself as an include path
    list(APPEND MODULES_INCLUDES ${MOD_DIR})

    set(MODULES_SOURCES "${MODULES_SOURCES}" PARENT_SCOPE)
    set(MODULES_INCLUDES "${MODULES_INCLUDES}" PARENT_SCOPE)
endfunction()

file(GLOB CHILD_DIRS LIST_DIRECTORIES TRUE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
foreach(child ${CHILD_DIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
        register_module(${child})
    endif()
endforeach()

# Now create the library with all collected sources
add_library(modules STATIC ${MODULES_SOURCES})

# Add all module directories as include paths
target_include_directories(modules PUBLIC ${MODULES_INCLUDES})