#!/usr/bin/env bash
# Run inside the dev container after colcon build, or via:
#   docker compose run --rm dev bash /workspaces/turtlebot3_behavior_demos/scripts/verify-cross-platform.sh
set -euo pipefail

REPO="${REPO:-/workspaces/turtlebot3_behavior_demos}"
VERIFY="${REPO}/docker/verify-ortools-build.sh"

if [[ ! -f "${VERIFY}" ]]; then
  REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  VERIFY="${REPO}/docker/verify-ortools-build.sh"
fi

if [[ -f /entrypoint.sh ]]; then
  # shellcheck disable=SC1091
  source /entrypoint.sh
fi

if [[ ! -d /overlay_ws/install/gpr_planning ]]; then
  echo "Building gpr_planning in /overlay_ws ..."
  cd /overlay_ws
  colcon build --symlink-install --packages-select gpr_planning
fi

bash "${VERIFY}"

echo ""
echo "Cross-platform checks passed for $(uname -m)."
echo "GitHub Actions also builds linux/arm64 via QEMU — see .github/workflows/multiarch.yml"
