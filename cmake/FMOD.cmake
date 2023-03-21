
macro(import_fmod_target target directory)
    add_library(${target} STATIC IMPORTED)
    # target_include_directories(${target} INTERFACE "${directory}/inc/")
    set_target_properties(${target} PROPERTIES INCLUDE_DIRECTORIES "${directory}/inc/")
    set_target_properties(${target} PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${directory}/inc/")
    set_target_properties(${target} PROPERTIES IMPORTED_LOCATION "${directory}/lib/x64/${target}_vc.lib")
endmacro()

if(MSVC)
    get_filename_component(fmodApiDirectory "[HKEY_CURRENT_USER\\Software\\FMOD Studio API Windows]" ABSOLUTE CACHE)
    set(fmodApiDirectory "${fmodApiDirectory}/api/")
    string(REPLACE "\\" "/" fmodApiDirectory "${fmodApiDirectory}")
endif()
if(EXISTS "${fmodApiDirectory}")
    set(fmodAvailable True)
    message("hit")
    import_fmod_target(fmod "${fmodApiDirectory}/core/")
    import_fmod_target(fsbank "${fmodApiDirectory}/fsbank/")
    import_fmod_target(fmodstudio "${fmodApiDirectory}/studio/")
endif()

message("fmodAvailable : ${fmodAvailable}")
message("fmodApiDirectory : ${fmodApiDirectory}")

get_target_property(fmodIncludeDirectories fmod INTERFACE_INCLUDE_DIRECTORIES)
message("fmod INTERFACE_INCLUDE_DIRECTORIES ${fmodIncludeDirectories}")
get_target_property(fsbankIncludeDirectories fsbank INTERFACE_INCLUDE_DIRECTORIES)
message("fsbank INTERFACE_INCLUDE_DIRECTORIES ${fsbankIncludeDirectories}")
get_target_property(fmodstudioIncludeDirectories fmodstudio INTERFACE_INCLUDE_DIRECTORIES)
message("fmodstudio INTERFACE_INCLUDE_DIRECTORIES ${fmodstudioIncludeDirectories}")
