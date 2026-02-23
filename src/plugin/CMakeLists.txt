set(SIMCORE_BUILD_SDF_PLUGIN_DEFAULT ON)
option(SIMCORE_BUILD_SDF_PLUGIN "Build built-in SDF plugins as a shared library" ${SIMCORE_BUILD_SDF_PLUGIN_DEFAULT})

if(SIMCORE_BUILD_SDF_PLUGIN)
  add_subdirectory(sdf)
endif()
