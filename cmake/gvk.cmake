
include(FetchContent)
set(GVK_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_SAMPLES_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_IDE_FOLDER "${DST_IDE_FOLDER}/external/gvk" CACHE STRING "" FORCE)
FetchContent_Declare(
    gvk
    GIT_REPOSITORY "https://github.com/dynamic-static/gvk.git"
    GIT_TAG 1bf328c3182cc007d893b96226bf076124d8bbb6
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(gvk)
