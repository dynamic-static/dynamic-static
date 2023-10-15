
################################################################################
# dynamic-static.graphics
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static.graphics/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static.graphics/")
dst_add_static_library(
    target
        dynamic-static.graphics
    linkLibraries
        dynamic-static.core
        gvk
        stb
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/buffer.hpp"
        "${includePath}/line-renderer.hpp"
        "${includePath}/mesh.hpp"
        "${includePath}/defines.hpp"
        "${includePath}/primitives.hpp"
        "${includePath}/renderer.hpp"
        "${includePath}/sprite-renderer.hpp"
        "${includePath}/sprite.hpp"
        "${includePath}/text.hpp"
    sourceFiles
        "${sourcePath}/buffer.cpp"
        "${sourcePath}/line-renderer.cpp"
        "${sourcePath}/mesh.cpp"
        "${sourcePath}/placeholder.cpp"
        "${sourcePath}/renderer.cpp"
        "${sourcePath}/sprite-renderer.cpp"
        "${sourcePath}/sprite.cpp"
        "${sourcePath}/text.cpp"
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
