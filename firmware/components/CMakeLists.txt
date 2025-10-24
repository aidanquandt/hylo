# ============================================================================ 
# Platform Include Directories (Auto-discovers all platform subdirectories)
# ============================================================================
set(PLATFORM_SOURCES "")
set(PLATFORM_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/platform)

# Add main platform source
list(APPEND PLATFORM_SOURCES platform/platform.c)

# Scan platform directory for subdirectories
file(GLOB PLATFORM_SUBDIRS LIST_DIRECTORIES TRUE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/platform ${CMAKE_CURRENT_SOURCE_DIR}/platform/*)

foreach(subdir_name ${PLATFORM_SUBDIRS})
    set(SUBDIR_PATH ${CMAKE_CURRENT_SOURCE_DIR}/platform/${subdir_name})
    if(IS_DIRECTORY ${SUBDIR_PATH})
        # Collect sources from each platform subdirectory
        file(GLOB SUBDIR_SOURCES CONFIGURE_DEPENDS
            ${SUBDIR_PATH}/*.c
            ${SUBDIR_PATH}/*.cpp
            ${SUBDIR_PATH}/*.cxx
        )

        if(SUBDIR_SOURCES)
            list(APPEND PLATFORM_SOURCES ${SUBDIR_SOURCES})
        endif()

        # Add the subdirectory as an include path
        list(APPEND PLATFORM_INCLUDES ${SUBDIR_PATH})
    endif()
endforeach()

# ============================================================================
# Modules Include Directories (Auto-discovers all module subdirectories)
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
        ${CMAKE_CURRENT_SOURCE_DIR}/includes
        ${PLATFORM_INCLUDES}
        ${MODULES_INCLUDES}
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
        ${PLATFORM_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/includes
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
        ${MODULES_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/includes
        ${PLATFORM_INCLUDES}
)

target_link_libraries(modules
    PUBLIC
        stm32cubemx
)