
macro(import_fmod_target target directory)
    add_library(${target} STATIC IMPORTED)
    set_target_properties(${target} PROPERTIES INCLUDE_DIRECTORIES "${directory}/inc/")
    set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${directory}/inc/")
    set_target_properties(${target} PROPERTIES IMPORTED_LOCATION "${directory}/lib/x64/${target}_vc.lib")
    set(${target}_SHARED_LIBRARY "${directory}/lib/x64/${target}.dll")
endmacro()

if(MSVC)
    cmake_host_system_information(RESULT fmodApiDirectory QUERY WINDOWS_REGISTRY "HKEY_CURRENT_USER\\Software\\FMOD Studio API Windows")
    set(fmodApiDirectory "${fmodApiDirectory}/api/")
    string(REPLACE "\\" "/" fmodApiDirectory "${fmodApiDirectory}")
endif()
if(EXISTS "${fmodApiDirectory}")
    set(fmodAvailable True)
    import_fmod_target(fmod "${fmodApiDirectory}/core/")
    import_fmod_target(fsbank "${fmodApiDirectory}/fsbank/")
    import_fmod_target(fmodstudio "${fmodApiDirectory}/studio/")
    get_target_property(fmodIncludeDirectories fmod INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(fsbankIncludeDirectories fsbank INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(fmodstudioIncludeDirectories fmodstudio INTERFACE_INCLUDE_DIRECTORIES)
    list(APPEND fmodSharedLibraries ${fmod_SHARED_LIBRARY} ${fsbank_SHARED_LIBRARY} ${fmodstudio})
else()
    set(DST_FMOD_ENABLED OFF CACHE BOOL "" FORCE)
    set(DST_AUDIO_ENABLED OFF CACHE BOOL "" FORCE)
endif()

# TODO : Documentation
# Create free FMOD account
# Goto Download
# Select [FMOD Engine "For integration of the FMOD run-time with custom engines"]
# Default recent-stable version should be fine
# Select platform specific download
#   On Windows, run the installer default options should be fine, restart required
