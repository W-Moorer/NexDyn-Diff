set(SIMCORE_SDF_PLUGIN_SRCS
    gear_plugin.cc
)

add_library(simcore_sdf_plugin SHARED ${SIMCORE_SDF_PLUGIN_SRCS})
target_compile_features(simcore_sdf_plugin PRIVATE cxx_std_17)
target_include_directories(
  simcore_sdf_plugin
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(simcore_sdf_plugin PRIVATE simcore)
