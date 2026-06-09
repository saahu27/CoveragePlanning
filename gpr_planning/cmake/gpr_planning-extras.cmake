# Static libgpr_planning_core.a embeds OR-Tools symbols when GPR_HAS_ORTOOLS=1, so CMake
# exports ortools::ortools on the target interface. Ensure the imported target exists
# before consumers load export_gpr_planningExport.cmake.
if(NOT TARGET ortools::ortools)
  set(_gpr_ortools_prefix "/opt/ortools")
  if(EXISTS "${_gpr_ortools_prefix}/lib/cmake/ortools/ortoolsConfig.cmake")
    list(PREPEND CMAKE_PREFIX_PATH "${_gpr_ortools_prefix}")
    find_package(ortools CONFIG REQUIRED)
  endif()
  unset(_gpr_ortools_prefix)
endif()
