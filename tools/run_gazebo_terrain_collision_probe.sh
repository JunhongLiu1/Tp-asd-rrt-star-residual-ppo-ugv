#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u
exec python3 "${HOME}/terrain_radiation_ws/tools/gazebo_terrain_collision_probe.py" "$@"
