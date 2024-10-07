
################################################################################
# dynamic-static.audio
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static.audio/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static.audio/")
dst_add_static_library(
    target
        dynamic-static.audio
    linkLibraries
        fmod
        fsbank
        fmodstudio
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/context.hpp"
        "${includePath}/defines.hpp"
    sourceFiles
        "${sourcePath}/context.cpp"
)

################################################################################
# dynamic-static.audio.test
set(testsPath "${CMAKE_CURRENT_LIST_DIR}/tests/")
dst_add_target_test(
    target
        dynamic-static.audio
    sourceFiles
        "${testsPath}/placeholder.tests.cpp"
)
