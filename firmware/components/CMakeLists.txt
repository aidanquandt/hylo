# Consolidated CMakeLists for all components

# ============================================================================
# App Library
# ============================================================================
add_library(app STATIC
    app/app.c
)

target_include_directories(app
    PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(app
    PUBLIC
        stm32cubemx
)

# ============================================================================
# Platform Library
# ============================================================================
add_library(platform STATIC
    platform/platform.c
)

target_include_directories(platform
    PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(platform
    PUBLIC
        stm32cubemx
)

# ============================================================================
# Modules Library (Auto-discovers all module subdirectories)
# ============================================================================
set(MODULES_SOURCES "")
set(MODULES_INCLUDES "")

# Scan modules directory for subdirectories
file(GLOB MODULE_DIRS LIST_DIRECTORIES TRUE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/modules ${CMAKE_CURRENT_SOURCE_DIR}/modules/*)

foreach(module_name ${MODULE_DIRS})
    set(MOD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/modules/${module_name})
    if(IS_DIRECTORY ${MOD_DIR})
        # Collect sources from each module
        file(GLOB MODULE_SOURCES CONFIGURE_DEPENDS
            ${MOD_DIR}/*.c
            ${MOD_DIR}/*.cpp
            ${MOD_DIR}/*.cxx
        )

        if(MODULE_SOURCES)
            list(APPEND MODULES_SOURCES ${MODULE_SOURCES})
        endif()

        # Add the module directory as an include path
        list(APPEND MODULES_INCLUDES ${MOD_DIR})
    endif()
endforeach()

# Create the modules library with all collected sources
add_library(modules STATIC ${MODULES_SOURCES})

# Add all module directories as include paths
target_include_directories(modules PUBLIC ${MODULES_INCLUDES})
