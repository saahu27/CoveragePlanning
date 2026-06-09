#!/usr/bin/env bash
# Fail the Docker/CI build if OR-Tools was not linked into gpr_planning.
# Safe to run after: colcon build in /overlay_ws (overlay or dev image).
set -euo pipefail

ORTOOLS_CMAKE="/opt/ortools/lib/cmake/ortools/ortoolsConfig.cmake"
BUILD_ROOT="${BUILD_ROOT:-/overlay_ws/build/gpr_planning}"
INSTALL_LIB="${INSTALL_LIB:-/overlay_ws/install/gpr_planning/lib/libgpr_planning_core.so}"

echo "=== OR-Tools build verification ==="
echo "machine: $(uname -m)"
echo "dpkg arch: $(dpkg --print-architecture 2>/dev/null || echo n/a)"

if [[ ! -f "${ORTOOLS_CMAKE}" ]]; then
  echo "FAIL: OR-Tools not installed at /opt/ortools" >&2
  exit 1
fi
echo "OK: ${ORTOOLS_CMAKE}"

mapfile -t ortools_objects < <(find "${BUILD_ROOT}" -name 'or_tools_atsp_solver.cpp.o' 2>/dev/null || true)
if [[ ${#ortools_objects[@]} -eq 0 ]]; then
  echo "FAIL: or_tools_atsp_solver.cpp.o not found under ${BUILD_ROOT}" >&2
  echo "      gpr_planning was built with heuristic ATSP only (GPR_HAS_ORTOOLS=0)." >&2
  exit 1
fi
echo "OK: compiled OrToolsAtspSolver (${ortools_objects[0]})"

if [[ -f "${INSTALL_LIB}" ]]; then
  if command -v nm >/dev/null 2>&1; then
    if nm -D "${INSTALL_LIB}" 2>/dev/null | grep -q 'operations_research::RoutingModel'; then
      echo "OK: OR-Tools RoutingModel symbol present in ${INSTALL_LIB}"
    else
      echo "WARN: RoutingModel symbol not visible in ${INSTALL_LIB} (static link may hide it)"
    fi
  fi
else
  echo "WARN: ${INSTALL_LIB} not found (skipping nm check)"
fi

echo "=== OR-Tools verification passed on $(uname -m) ==="
