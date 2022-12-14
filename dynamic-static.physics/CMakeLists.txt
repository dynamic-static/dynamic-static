
################################################################################
# dynamic-static.physics
set(includeDirectory "${CMAKE_CURRENT_LIST_DIR}/include/")
set(includePath "${includeDirectory}/dynamic-static.physics/")
set(sourcePath "${CMAKE_CURRENT_LIST_DIR}/source/dynamic-static.physics/")
dst_add_static_library(
    target
        dynamic-static.physics
    linkLibraries
        Bullet2FileLoader
        Bullet3Collision
        Bullet3Common
        Bullet3Dynamics
        Bullet3Geometry
        Bullet3OpenCL_clew
        BulletCollision
        BulletDynamics
        BulletInverseDynamics
        BulletSoftBody
        LinearMath
    includeDirectories
        "${includeDirectory}"
    includeFiles
        "${includePath}/defines.hpp"
        "${includePath}/context.hpp"
    sourceFiles
        "${sourcePath}/context.cpp"
)

################################################################################
# dynamic-static.physics.test
set(testsPath "${CMAKE_CURRENT_LIST_DIR}/tests/")
dst_add_target_test(
    target
        dynamic-static.physics
    sourceFiles
        "${testsPath}/placeholder.tests.cpp"
)
