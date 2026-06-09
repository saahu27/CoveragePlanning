#!/usr/bin/env bash
# Runs on the host before the dev container is built or started.
set -euo pipefail

# Allow container processes to use the host X display (Gazebo, RViz, OpenCV windows).
xhost +local:docker 2>/dev/null || true

mkdir -p .colcon/build .colcon/install .colcon/log

echo "Host initialized for GPR coverage dev container (X11 + colcon directories)."
