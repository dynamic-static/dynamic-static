
################################################################################
# dynamic-static.core
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static/")
dst_add_static_library(
    target
        dynamic-static.core
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/defines.hpp"
    sourceFiles
        "${sourcePath}/placeholder.cpp"
)

################################################################################
# dynamic-static.core.test
set(testsPath "${CMAKE_CURRENT_LIST_DIR}/tests/")
dst_add_target_test(
    target
        dynamic-static.core
    sourceFiles
        "${testsPath}/placeholder.tests.cpp"
)
