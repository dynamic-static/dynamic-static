
include(FetchContent)
set(GVK_BUILD_COMMAND_STRUCTURES OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_LAYER              OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_STATE_TRACKER      OFF CACHE BOOL "" FORCE)
set(GVK_BUILD_TESTS              OFF CACHE BOOL "" FORCE)
set(GVK_IDE_FOLDER "${DST_IDE_FOLDER}/external/gvk" CACHE STRING "" FORCE)
FetchContent_Declare(
    gvk
    GIT_REPOSITORY "https://github.com/dynamic-static/gvk.git"
    GIT_TAG dba8ca487b48d886ff47b9103948e776240d06fe
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(gvk)
