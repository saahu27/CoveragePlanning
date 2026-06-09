#!/usr/bin/env bash
# Runs once after the dev container is created.
set -eo pipefail

# Prefer repo entrypoint (mounted) over image copy so script fixes apply without rebuild.
ENTRYPOINT_SH="/workspaces/turtlebot3_behavior_demos/docker/entrypoint.sh"
if [[ ! -f "${ENTRYPOINT_SH}" ]]; then
  ENTRYPOINT_SH="/entrypoint.sh"
fi
# shellcheck disable=SC1090
source "${ENTRYPOINT_SH}"

mkdir -p /overlay_ws/build /overlay_ws/install /overlay_ws/log

cd /overlay_ws
colcon build --symlink-install

echo "Overlay workspace built at /overlay_ws."
