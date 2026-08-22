#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
source /home/i/terrain_radiation_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/home/i/terrain_radiation_ws/acceptance_logs/m1_m3_gazebo_20260822
mkdir -p "$ROS_LOG_DIR/runtime"
cd /tmp

ros2 launch radiation_mapping terrain_services.launch.py \
  terrain:=hard use_sim_time:=true \
  > "$ROS_LOG_DIR/runtime/terrain_services_console.log" 2>&1 &
terrain_pid=$!

cleanup() {
  kill -INT "$terrain_pid" 2>/dev/null || true
  wait "$terrain_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch risk_aware_planner_cpp tp_asd_rrt_star_online_radiation.launch.py \
  enable_motion:=true enable_frame_alignment:=false \
  metrics_csv:=/home/i/terrain_radiation_ws/acceptance_logs/m1_m3_gazebo_20260822/planner_metrics.csv
