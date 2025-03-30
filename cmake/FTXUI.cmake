
include_guard()

include(FetchContent)

set(FTXUI_VERSION 2c9a828402a6feca4310ab1f20b8c428901803a5) # main 22-March-2025
FetchContent_Declare(
    FTXUI
    GIT_REPOSITORY "https://github.com/ArthurSonzogni/FTXUI.git"
    GIT_TAG ${FTXUI_VERSION}
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(FTXUI)
set_target_properties(FTXUI PROPERTIES FOLDER "${GVK_IDE_FOLDER}/external/")
