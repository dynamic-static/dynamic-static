
include(FetchContent)
set(GVK_TESTS_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_SAMPLES_ENABLED OFF CACHE BOOL "" FORCE)
set(GVK_IDE_FOLDER "${DST_IDE_FOLDER}/external/gvk" CACHE STRING "" FORCE)
FetchContent_Declare(
    gvk
    GIT_REPOSITORY "https://github.com/dynamic-static/gvk.git"
    GIT_TAG 1c7e3f4d31389c3753ee1e0a37611693dc97fd54
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(gvk)
