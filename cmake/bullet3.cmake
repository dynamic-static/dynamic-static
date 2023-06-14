
include_guard()

include(FetchContent)

set(USE_MSVC_RUNTIME_LIBRARY_DLL ON)
FetchContent_Declare(
    bullet3
    GIT_REPOSITORY "https://github.com/dynamic-static/bullet3.git"
    GIT_TAG 1f6bcc3ff9394163cc8fae84a982ad9ee66aec77 # 3.24 + tksuoran-cmake-fetchcontent
    GIT_PROGRESS TRUE
    FETCHCONTENT_UPDATES_DISCONNECTED
)
FetchContent_MakeAvailable(bullet3)
set_target_properties(Bullet2FileLoader PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(Bullet3Collision PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(Bullet3Common PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(Bullet3Dynamics PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(Bullet3Geometry PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(Bullet3OpenCL_clew PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(BulletCollision PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(BulletDynamics PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(BulletInverseDynamics PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(BulletSoftBody PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
set_target_properties(LinearMath PROPERTIES FOLDER "${DST_IDE_FOLDER}/external/bullet3/")
