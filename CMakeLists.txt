cmake_minimum_required(VERSION 3.16)

# Keep the project name as `simcore` so copied CMake modules that reference
# `SIMCORE_SOURCE_DIR` continue to work without invasive edits.
project(
  simcore
  VERSION 1.0.0
  DESCRIPTION "SimCore (simcore core-only migration)"
)

set(SIMCORE_SOURCE_DIR "${PROJECT_SOURCE_DIR}")
set(SIMCORE_CMAKE_MIN_REQ "3.16")

set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

enable_language(C)
enable_language(CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Core-only migration profile: disable non-core targets by default.
set(BUILD_TESTING OFF CACHE BOOL "Disable tests in migrated core build" FORCE)
set(SIMCORE_BUILD_TESTS OFF CACHE BOOL "Disable simcore tests" FORCE)
set(SIMCORE_BUILD_EXAMPLES OFF CACHE BOOL "Disable samples" FORCE)
set(SIMCORE_BUILD_SIMULATE OFF CACHE BOOL "Disable simulate" FORCE)
set(SIMCORE_BUILD_STUDIO OFF CACHE BOOL "Disable studio" FORCE)
set(SIMCORE_USE_FILAMENT OFF CACHE BOOL "Disable filament" FORCE)
set(SIMCORE_WITH_USD OFF CACHE BOOL "Disable OpenUSD integration" FORCE)

include(SimcoreOptions)
include(SimcoreMacOS)
include(SimcoreDependencies)

set(SIMCORE_CORE_HEADERS
    include/simcore/core_api.h
    include/simcore/SIM_data.h
    include/simcore/SIM_export.h
    include/simcore/SIM_macro.h
    include/simcore/SIM_model.h
    include/simcore/SIM_plugin.h
    include/simcore/SIM_san.h
    include/simcore/SIM_spec.h
    include/simcore/SIM_thread.h
    include/simcore/SIM_tnum.h
    include/simcore/SIM_xmacro.h
    include/simcore/base.h
    include/simcore/foundation.h
    include/simcore/math.h
    include/simcore/spec.h
    include/simcore/xml.h
    include/simcore/model.h
    include/simcore/data.h
    include/simcore/dynamics.h
    include/simcore/collision.h
    include/simcore/solver.h
    include/simcore/plugin.h
    include/simcore/resource.h
    include/simcore/runtime.h
    include/simcore/simcore.h
)

add_library(simcore SHARED)
add_library(simcore::core ALIAS simcore)

target_include_directories(
  simcore
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE src
)

add_subdirectory(src/engine)
add_subdirectory(src/user)
add_subdirectory(src/xml)
add_subdirectory(src/thread)
add_subdirectory(src/plugin)

target_compile_definitions(
  simcore
  PRIVATE _GNU_SOURCE
          CCD_STATIC_DEFINE
          SIMCORE_DLL_EXPORTS
          -DMC_IMPLEM_ENABLE
)
if(SIMCORE_ENABLE_AVX_INTRINSICS)
  target_compile_definitions(simcore PUBLIC SIM_USEPLATFORMSIMD)
endif()

target_compile_options(
  simcore
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${SIMCORE_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${SIMCORE_CXX_FLAGS}
)
target_link_options(
  simcore
  PRIVATE ${SIMCORE_MACOS_LINK_OPTIONS}
          ${EXTRA_LINK_OPTIONS}
)

target_link_libraries(
  simcore
  PRIVATE ccd
          lodepng
          qhullstatic_r
          tinyobjloader
          tinyxml2
)

option(SIMCORE_BUILD_SMOKE "Build minimal simcore API smoke executable" OFF)
if(SIMCORE_BUILD_SMOKE)
  add_executable(simcore_p1_smoke tests/p1_smoke.c)
  target_include_directories(simcore_p1_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p1_smoke PRIVATE simcore)

  add_executable(simcore_p2_lifecycle_smoke tests/p2_lifecycle_smoke.c)
  target_include_directories(simcore_p2_lifecycle_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p2_lifecycle_smoke PRIVATE simcore)

  add_executable(simcore_p3_dynamics_solver_smoke tests/p3_dynamics_solver_smoke.c)
  target_include_directories(simcore_p3_dynamics_solver_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p3_dynamics_solver_smoke PRIVATE simcore)

  add_executable(simcore_p4_contact_smoke tests/p4_contact_smoke.c)
  target_include_directories(simcore_p4_contact_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p4_contact_smoke PRIVATE simcore)

  add_executable(simcore_p5_modeling_smoke tests/p5_modeling_smoke.c)
  target_include_directories(simcore_p5_modeling_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p5_modeling_smoke PRIVATE simcore)

  add_executable(simcore_p6_extension_smoke tests/p6_extension_smoke.c)
  target_include_directories(simcore_p6_extension_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p6_extension_smoke PRIVATE simcore)

  add_executable(simcore_p7_foundation_math_smoke tests/p7_foundation_math_smoke.c)
  target_include_directories(simcore_p7_foundation_math_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_p7_foundation_math_smoke PRIVATE simcore)
endif()

option(SIMCORE_BUILD_GEAR_EXPORT_CPP "Build C++ gear CSV export executable" ON)
if(SIMCORE_BUILD_GEAR_EXPORT_CPP)
  add_executable(simcore_gear_export tools/gear_contact_export_cpp.cc)
  target_compile_features(simcore_gear_export PRIVATE cxx_std_17)
  target_include_directories(simcore_gear_export PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(simcore_gear_export PRIVATE simcore)
endif()

option(SIMCORE_BUILD_DOUBLE_PENDULUM_SIM "Build double_pendulum simulation tool" ON)
if(SIMCORE_BUILD_DOUBLE_PENDULUM_SIM)
  add_executable(double_pendulum_sim tools/double_pendulum_sim.cc)
  target_compile_features(double_pendulum_sim PRIVATE cxx_std_17)
  target_include_directories(double_pendulum_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(double_pendulum_sim PRIVATE simcore)
endif()

set_target_properties(
  simcore
  PROPERTIES OUTPUT_NAME "simcore"
             VERSION "${PROJECT_VERSION}"
             PUBLIC_HEADER "${SIMCORE_CORE_HEADERS}"
)
