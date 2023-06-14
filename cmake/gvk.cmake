
include_guard()

include(FetchContent)

set(GVK_BUILD_COMMAND_STRUCTURES OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_LAYER              OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_STATE_TRACKER      OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_TESTS              OFF CACHE BOOL "" FORCE)
set(GVK_IDE_FOLDER "${DST_IDE_FOLDER}/external/gvk" CACHE STRING "" FORCE)
FetchContent_Declare(
    gvk
    GIT_REPOSITORY "https://github.com/dynamic-static/gvk.git"
    GIT_TAG 3ba52a5def2f9d87bda712214a12cbb2e5c7524f
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(gvk)
