
include_guard()

include(FetchContent)

FetchContent_Declare(
    stb_truetype
    GIT_REPOSITORY "https://github.com/nothings/stb.git"
    GIT_TAG 5736b15f7ea0ffb08dd38af21067c314d6a3aae9
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(stb_truetype)
FetchContent_GetProperties(stb_truetype SOURCE_DIR stbSourceDirectory)
FetchContent_GetProperties(stb_truetype BINARY_DIR stbBinaryDirectory)

macro(add_stb_file stbFile implementationMacro)
    file(COPY "${stbSourceDirectory}/${stbFile}.h" DESTINATION "${stbBinaryDirectory}/stb/")
    list(APPEND includeFiles "${stbBinaryDirectory}/stb/${stbFile}.h")
    set(sourceFile "${stbBinaryDirectory}/stb/${stbFile}.cpp")
    list(APPEND sourceFiles "${sourceFile}")
    if(NOT EXISTS "${sourceFile}")
        file(WRITE "${sourceFile}"
"
#ifdef _MSVC_LANG
#pragma warning(push, 0)
#endif
#define ${implementationMacro}
#include \"${stbFile}.h\"
#ifdef _MSVC_LANG
#pragma warning(pop)
#endif
"
        )
    endif()
endmacro()

add_stb_file(stb_truetype STB_TRUETYPE_IMPLEMENTATION)

dst_add_static_library(
    target stb_truetype
    folder "external/"
    includeDirectories "${stbBinaryDirectory}/"
    includeFiles "${includeFiles}"
    sourceFiles "${sourceFiles}"
)
