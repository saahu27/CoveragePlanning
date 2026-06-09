#!/bin/bash
# Basic entrypoint for ROS Docker containers.
# Safe to source from scripts (e.g. post-create.sh) or run as the container entrypoint.

ros_entrypoint_setup() {
  if [[ -f /etc/profile.d/gpr-puppeteer.sh ]]; then
    # shellcheck disable=SC1091
    source /etc/profile.d/gpr-puppeteer.sh
  fi

  # ROS setup.bash reads optional vars (e.g. AMENT_TRACE_SETUP_FILES) that may be unset.
  local restore_nounset=false
  if [[ $- == *u* ]]; then
    restore_nounset=true
    set +u
  fi

  source /opt/ros/${ROS_DISTRO}/setup.bash

  if [ -f /turtlebot_ws/install/setup.bash ]; then
    source /turtlebot_ws/install/setup.bash
  fi

  if [ -f /overlay_ws/install/setup.bash ]; then
    source /overlay_ws/install/setup.bash
  fi

  if ${restore_nounset}; then
    set -u
  fi
}

ros_entrypoint_setup

# Only exec when run as the container entrypoint, not when sourced by dev scripts.
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Sourced ROS 2 ${ROS_DISTRO}"
  if [ -f /turtlebot_ws/install/setup.bash ]; then
    echo "Sourced TurtleBot base workspace"
  fi
  if [ -f /overlay_ws/install/setup.bash ]; then
    echo "Sourced GPR overlay workspace"
  fi
  exec "$@"
fi
