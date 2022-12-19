
include(FetchContent)
set(GVK_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_SAMPLES_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_IDE_FOLDER "${DST_IDE_FOLDER}/external/gvk" CACHE STRING "" FORCE)
FetchContent_Declare(
    gvk
    GIT_REPOSITORY "https://github.com/dynamic-static/gvk.git"
    GIT_TAG 1c7dd005c105f437bd58de839206266397578920
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(gvk)
