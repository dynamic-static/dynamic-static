
################################################################################
# dynamic-static.graphics
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static.graphics/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static.graphics/")
dst_add_static_library(
    target
        dynamic-static.graphics
    linkLibraries
        gvk
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/defines.hpp"
        "${includePath}/primitives.hpp"
    sourceFiles
        "${sourcePath}/primitives.cpp"
)

################################################################################
# dynamic-static.graphics.test
set(testsPath "${CMAKE_CURRENT_LIST_DIR}/tests/")
dst_add_target_test(
    target
        dynamic-static.graphics
    sourceFiles
        "${testsPath}/placeholder.tests.cpp"
)
