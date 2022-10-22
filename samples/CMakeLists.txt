
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

dst_add_sample(placeholder)
