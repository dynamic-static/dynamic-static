
macro(dst_add_sample target)
    dst_add_executable(
        target ${target}
        folder "samples/"
        linkLibraries dynamic-static
        sourceFiles
            "${CMAKE_CURRENT_LIST_DIR}/dynamic-static.sample-utilities.hpp"
            "${CMAKE_CURRENT_LIST_DIR}/${target}.cpp"
    )
endmacro()

# TODO : Documentation
dst_add_sample(brick-breaker)

# TODO : Documentation
dst_add_sample(shape-shooter)
FetchContent_Declare(
    MonoGameSamples
    GIT_REPOSITORY "https://github.com/MonoGame/MonoGame.Samples.git"
    GIT_TAG b277067deda3dd57fe67b4927f6df305de12cd06 # 3.8.1
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(MonoGameSamples)
FetchContent_GetProperties(MonoGameSamples SOURCE_DIR MonoGameSamples_SOURCE_DIR)
target_compile_definitions(shape-shooter PRIVATE SHAPE_SHOOTER_CONTENT="${MonoGameSamples_SOURCE_DIR}/NeonShooter/NeonShooter.Core/Content/")
# add_custom_command(
#     TARGET shape-shooter POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#     ${fmodSharedLibraries}
#     $<TARGET_FILE_DIR:shape-shooter>
# )
