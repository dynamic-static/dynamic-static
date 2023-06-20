
cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 20)
project(dynamic-static)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/")
include(dynamic-static.build)
set(DST_IDE_FOLDER "" CACHE STRING "")
if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
    set(DST_STANDALONE ON)
else()
    set(DST_STANDALONE OFF)
endif()

################################################################################
# Build options
include(CMakeDependentOption)
option                (DST_BUILD_TESTS           "" ${DST_STANDALONE})
cmake_dependent_option(DST_RUN_TESTS             "" ON "DST_BUILD_TESTS" OFF)
option                (DST_BULLET_ENABLED        "" ON)
option                (DST_FMOD_ENABLED          "" ON)
option                (DST_GVK_ENABLED           "" ON)
option                (DST_CORE_ENABLED          "" ON)
cmake_dependent_option(DST_AUDIO_ENABLED         "" ON "DST_CORE_ENABLED;DST_FMOD_ENABLED" OFF)
cmake_dependent_option(DST_GRAPHICS_ENABLED      "" ON "DST_CORE_ENABLED;DST_GVK_ENABLED" OFF)
cmake_dependent_option(DST_PHYSICS_ENABLED       "" ON "DST_CORE_ENABLED;DST_BULLET_ENABLED" OFF)
option                (DST_BUILD_SAMPLES         "" ${DST_STANDALONE})
option                (DST_CREATE_SOURCE_PACKAGE "" OFF)
cmake_dependent_option(DST_CREATE_TEST_PACKAGE   "" OFF "DST_BUILD_TESTS" OFF)

################################################################################
# Dependencies
macro(dst_include_depenedency enabled dependency)
    if(${enabled})
        include(${dependency})
    endif()
endmacro()

dst_include_depenedency(DST_BUILD_TESTS    googletest)
dst_include_depenedency(DST_BULLET_ENABLED bullet3)
dst_include_depenedency(DST_FMOD_ENABLED   FMOD)
dst_include_depenedency(DST_GVK_ENABLED    gvk)

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
dst_add_module(dynamic-static.audio    ${DST_AUDIO_ENABLED})
dst_add_module(dynamic-static.graphics ${DST_GRAPHICS_ENABLED})
dst_add_module(dynamic-static.physics  ${DST_PHYSICS_ENABLED})

################################################################################
# Samples
if(DST_BUILD_SAMPLES)
    add_subdirectory(samples)
endif()
