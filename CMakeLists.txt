
cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)
project(dynamic-static)
set(DST_IDE_FOLDER "" CACHE STRING "")
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(DST_STANDALONE TRUE)
else()
    set(DST_STANDALONE FALSE)
endif()

################################################################################
# Build options
include(CMakeDependentOption)
# Test options
option                (DST_BUILD_TESTS            "" ${DST_STANDALONE})
cmake_dependent_option(DST_RUN_TESTS              "" ON
    "DST_BUILD_TESTS" OFF
)
# Dependency options
option                (DST_GVK_ENABLED            "" ON)
option                (DST_BULLET_ENABLED         "" ON)
# Target options
option                (DST_CORE_ENABLED           "" ON)
cmake_dependent_option(DST_GRAPHICS_ENABLED       "" ON
    "DST_CORE_ENABLED;DST_GVK_ENABLED" OFF
)
cmake_dependent_option(DST_PHYSICS_ENABLED        "" ON
    "DST_CORE_ENABLED;DST_BULLET_ENABLED" OFF
)
# Samples options
option                (DST_BUILD_SAMPLES          "" ${DST_STANDALONE})
# Packaging options
option                (DST_CREATE_SOURCE_PACKAGE  "" OFF)
cmake_dependent_option(DST_CREATE_TEST_PACKAGE    "" OFF
    "DST_BUILD_TESTS" OFF
)

################################################################################
# CMake utilities
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/dynamic-static.build.cmake")

################################################################################
# Dependencies
if(DST_TESTS_ENABLED)
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/googletest.cmake")
endif()
if(DST_GVK_ENABLED)
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/gvk.cmake")
endif()
if(DST_BULLET_ENABLED)
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/bullet3.cmake")
endif()

################################################################################
# Modules
add_library(dynamic-static INTERFACE)
macro(dst_add_module module enabled)
    if(${enabled})
        add_subdirectory(${module})
        target_link_libraries(dynamic-static INTERFACE ${module})
    endif()
endmacro()

dst_add_module(dynamic-static.core     ${DST_CORE_ENABLED})
dst_add_module(dynamic-static.graphics ${DST_GRAPHICS_ENABLED})
dst_add_module(dynamic-static.physics  ${DST_PHYSICS_ENABLED})

################################################################################
# Samples
if(DST_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()
