# ============================================================================ 
# Auto-discover all subdirectories recursively for includes and sources
# ============================================================================

# Function to recursively find all subdirectories
function(get_all_subdirs result directory)
    set(subdirs "")
    file(GLOB_RECURSE all_files LIST_DIRECTORIES TRUE "${directory}/*")
    foreach(item ${all_files})
        if(IS_DIRECTORY ${item})
            list(APPEND subdirs ${item})
        endif()
    endforeach()
    set(${result} ${subdirs} PARENT_SCOPE)
endfunction()

# Collect all include directories from all component folders
set(ALL_COMPONENT_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${CMAKE_CURRENT_SOURCE_DIR}/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
)

# Recursively add all subdirectories
get_all_subdirs(APP_SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/app")
get_all_subdirs(CONFIG_SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/config")
get_all_subdirs(DRIVERS_SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/drivers")
get_all_subdirs(MODULES_SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/modules")
get_all_subdirs(PLATFORM_SUBDIRS "${CMAKE_CURRENT_SOURCE_DIR}/platform")

list(APPEND ALL_COMPONENT_INCLUDES ${APP_SUBDIRS} ${CONFIG_SUBDIRS} ${DRIVERS_SUBDIRS} ${MODULES_SUBDIRS} ${PLATFORM_SUBDIRS})

# ============================================================================ 
# Platform Sources (Auto-discovers all platform .c/.cpp files)
# ============================================================================
set(PLATFORM_SOURCES "")
file(GLOB_RECURSE PLATFORM_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/platform/*.cxx
)

# ============================================================================
# Modules Sources (Auto-discovers all module .c/.cpp files)
# ============================================================================
set(MODULES_SOURCES "")
file(GLOB_RECURSE MODULES_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/*.cxx
)

# ============================================================================
# Drivers Sources (Auto-discovers all driver .c/.cpp files)
# ============================================================================
set(DRIVERS_SOURCES "")
file(GLOB_RECURSE DRIVERS_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/*.cxx
)

# ============================================================================
# App Sources (Auto-discovers all app .c/.cpp files)
# ============================================================================
set(APP_SOURCES "")
file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/app/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/app/*.cxx
)

# ============================================================================
# App Library
# ============================================================================
add_library(app STATIC ${APP_SOURCES})

target_include_directories(app
    PUBLIC
        ${CMAKE_SOURCE_DIR}
        ${ALL_COMPONENT_INCLUDES}
)

target_link_libraries(app
    PUBLIC
        stm32cubemx
)

# ============================================================================ 
# Platform Library 
# ============================================================================ 
add_library(platform STATIC ${PLATFORM_SOURCES})

target_include_directories(platform
    PUBLIC
        ${ALL_COMPONENT_INCLUDES}
)

target_link_libraries(platform
    PUBLIC
        stm32cubemx
)

# ============================================================================
# Modules Library
# ============================================================================
add_library(modules STATIC ${MODULES_SOURCES})

target_include_directories(modules
    PUBLIC
        ${ALL_COMPONENT_INCLUDES}
)

target_link_libraries(modules
    PUBLIC
        stm32cubemx
        m
)

# ============================================================================
# Drivers Library
# ============================================================================
if(DRIVERS_SOURCES)
    add_library(drivers STATIC ${DRIVERS_SOURCES})

    target_include_directories(drivers
        PUBLIC
            ${ALL_COMPONENT_INCLUDES}
    )

    target_link_libraries(drivers
        PUBLIC
            stm32cubemx
    )
endif()