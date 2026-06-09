#!/usr/bin/env bash
# Runs each time the dev container starts.
set -eo pipefail

REPO="/workspaces/turtlebot3_behavior_demos"

if [[ -f "${REPO}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO}/.env"
  set +a
fi

if [[ "${DEVCONTAINER_AUTO_START_DEMO:-false}" == "true" ]]; then
  echo "DEVCONTAINER_AUTO_START_DEMO=true — launching GPR coverage..."
  bash -lc "cd /overlay_ws && source install/setup.bash && ros2 launch gpr_bringup gpr_coverage.launch.py"
else
  cat <<EOF

Dev container ready.

  Build + run GPR coverage:
    cd /overlay_ws && colcon build --symlink-install && source install/setup.bash
    ros2 launch gpr_bringup gpr_coverage.launch.py

  Docs / PDFs: docs/README.md  |  ./docs/build-docs.sh

  Multi-arch: image matches host CPU (amd64 or arm64). OR-Tools ATSP enabled on both.

EOF
fi
