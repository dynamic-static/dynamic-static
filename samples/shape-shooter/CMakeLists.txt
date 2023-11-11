
################################################################################
# TODO : Documentation
FetchContent_Declare(
    MonoGameSamples
    GIT_REPOSITORY "https://github.com/MonoGame/MonoGame.Samples.git"
    GIT_TAG b277067deda3dd57fe67b4927f6df305de12cd06 # 3.8.1
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(MonoGameSamples)
FetchContent_GetProperties(MonoGameSamples SOURCE_DIR MonoGameSamples_SOURCE_DIR)

################################################################################
# TODO : Documentation
set(SHAPE_SHOOTER_AUDIO_ENABLED 0)
if(${DST_FMOD_ENABLED})
    set(SHAPE_SHOOTER_AUDIO_ENABLED 1)
    add_custom_command(
        TARGET shape-shooter POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${fmodSharedLibraries}
        $<TARGET_FILE_DIR:shape-shooter>
    )
endif()

################################################################################
# shape-shooter
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/shape-shooter/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/shape-shooter/")
dst_add_executable(
    target
        shape-shooter
    folder
        "samples/"
    linkLibraries
        dynamic-static
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/bullet.hpp"
        "${includePath}/context.hpp"
        "${includePath}/defines.hpp"
        "${includePath}/enemy.hpp"
        "${includePath}/enemy-spawner.hpp"
        "${includePath}/entity.hpp"
        "${includePath}/entity-manager.hpp"
        "${includePath}/grid.hpp"
        "${includePath}/input-manager.hpp"
        "${includePath}/player-ship.hpp"
        "${includePath}/player-status.hpp"
        "${includePath}/utilities.hpp"
    sourceFiles
        "${sourcePath}/bullet.cpp"
        "${sourcePath}/context.cpp"
        "${sourcePath}/enemy.cpp"
        "${sourcePath}/enemy-spawner.cpp"
        "${sourcePath}/entity.cpp"
        "${sourcePath}/entity-manager.cpp"
        "${sourcePath}/grid.cpp"
        "${sourcePath}/input-manager.cpp"
        "${sourcePath}/player-ship.cpp"
        "${sourcePath}/player-status.cpp"
        "${sourcePath}/shape-shooter.cpp"
        "${sourcePath}/utilities.cpp"
    compileDefinitions
        SHAPE_SHOOTER_CONTENT="${MonoGameSamples_SOURCE_DIR}/NeonShooter/NeonShooter.Core/Content/"
        SHAPE_SHOOTER_AUDIO_ENABLED="${SHAPE_SHOOTER_AUDIO_ENABLED}"
)
