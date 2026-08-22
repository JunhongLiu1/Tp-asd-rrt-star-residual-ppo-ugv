#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source /home/i/terrain_radiation_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/home/i/terrain_radiation_ws/acceptance_logs/m1_m3_gazebo_20260822
mkdir -p "$ROS_LOG_DIR"
cd /tmp

exec ros2 launch radiation_mapping gazebo_radiation_husky_formal.launch.py gui:=false
