
################################################################################
# dynamic-static.core
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static/")
dst_add_static_library(
    target
        dynamic-static.core
    linkLibraries
        glm
        stb
        stb_truetype
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/bin-pack.hpp"
        "${includePath}/defines.hpp"
        "${includePath}/filesystem.hpp"
        "${includePath}/hash-combine.hpp"
        "${includePath}/image.hpp"
        "${includePath}/state-machine.hpp"
        "${includePath}/text.hpp"
        "${includePath}/type-id.hpp"
    sourceFiles
        "${sourcePath}/filesystem.cpp"
        "${sourcePath}/image.cpp"
        "${sourcePath}/placeholder.cpp"
        "${sourcePath}/state-machine.cpp"
        "${sourcePath}/text.cpp"
)

################################################################################
# dynamic-static.core.test
set(testsPath "${CMAKE_CURRENT_LIST_DIR}/tests/")
dst_add_target_test(
    target
        dynamic-static.core
    sourceFiles
        "${testsPath}/placeholder.tests.cpp"
        "${testsPath}/state-machine.tests.cpp"
)
