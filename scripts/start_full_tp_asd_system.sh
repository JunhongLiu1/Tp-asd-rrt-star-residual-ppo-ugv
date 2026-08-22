#!/usr/bin/env bash

set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/terrain_radiation_ws}"
ENABLE_MOTION="${ENABLE_MOTION:-false}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
GOAL_X="${GOAL_X:-0.0}"
GOAL_Y="${GOAL_Y:-0.0}"
METRICS_CSV="${METRICS_CSV:-/tmp/tp_asd_rrt_star_metrics.csv}"

source /opt/ros/foxy/setup.bash
source "$WORKSPACE/install/setup.bash"

LOG_ROOT="/tmp/tp_asd_rrt_star_startup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_ROOT"

PIDS=()
CLEANUP_DONE=0

kill_descendants() {
  local parent="$1"
  local child

  for child in $(pgrep -P "$parent" 2>/dev/null || true); do
    kill_descendants "$child"
    kill -INT "$child" 2>/dev/null || true
  done
}

cleanup() {
  if [ "$CLEANUP_DONE" -eq 1 ]; then
    return
  fi

  CLEANUP_DONE=1
  trap - EXIT INT TERM

  echo
  echo "Stopping complete process sessions..."

  for pid in "${PIDS[@]}"; do
    sid=$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')

    if [[ "$sid" =~ ^[0-9]+$ ]]; then
      pkill -INT -s "$sid" 2>/dev/null || true
    fi

    kill_descendants "$pid"
    kill -INT "$pid" 2>/dev/null || true
  done

  sleep 3

  for pid in "${PIDS[@]}"; do
    sid=$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')

    if [[ "$sid" =~ ^[0-9]+$ ]]; then
      pkill -TERM -s "$sid" 2>/dev/null || true
    fi

    kill -TERM "$pid" 2>/dev/null || true
  done

  sleep 2

  for pid in "${PIDS[@]}"; do
    sid=$(ps -o sid= -p "$pid" 2>/dev/null | tr -d ' ')

    if [[ "$sid" =~ ^[0-9]+$ ]]; then
      pkill -KILL -s "$sid" 2>/dev/null || true
    fi

    kill -KILL "$pid" 2>/dev/null || true
  done

  echo "Logs saved in: $LOG_ROOT"
}

trap cleanup EXIT INT TERM

start_process() {
  local name="$1"
  shift

  echo "Starting: $*"
  setsid "$@" >"$LOG_ROOT/${name}.log" 2>&1 &
  PIDS+=("$!")
}

wait_for_topic() {
  local topic="$1"
  local timeout_sec="$2"

  echo "Waiting for topic: $topic"

  for _ in $(seq 1 "$timeout_sec"); do
    if ros2 topic info "$topic" 2>/dev/null |
      grep -Eq "Publisher count: [1-9]"; then
      echo "Topic ready: $topic"
      return 0
    fi
    sleep 1
  done

  echo "ERROR: timeout waiting for $topic"
  return 1
}

echo "===== START FULL TP-ASD SYSTEM ====="
echo "Motion enabled: $ENABLE_MOTION"
echo "Goal: ($GOAL_X, $GOAL_Y)"
echo "CSV: $METRICS_CSV"

start_process \
  husky_dem \
  ros2 launch risk_aware_planner_cpp husky_dem.launch.py

sleep 8

echo "Starting terrain_layer_publisher"
start_process \
  terrain_layer_publisher \
  ros2 run radiation_mapping terrain_layer_publisher \
  --ros-args -p terrain_level:=hard

echo "Starting terrain_query_server"
start_process \
  terrain_query_server \
  ros2 run radiation_mapping terrain_query_server \
  --ros-args -p terrain_level:=hard

wait_for_topic /clock 60
wait_for_topic /radiation_map 60
wait_for_topic /terrain_impedance_map 60
wait_for_topic /terrain_traversability_mask 60

start_process \
  husky_control \
  ros2 launch husky_control control.launch.py

wait_for_topic /odometry/filtered 60

start_process \
  tp_asd_experiment \
  ros2 launch risk_aware_planner_cpp \
  tp_asd_rrt_star_online_radiation.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  enable_motion:="$ENABLE_MOTION" \
  metrics_csv:="$METRICS_CSV"

sleep 5

ros2 topic pub --once \
  /goal_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {w: 1.0}}}" \
  >"$LOG_ROOT/goal.log" 2>&1 || true

echo
echo "===== TP-ASD SYSTEM READY ====="
echo "World: module36_hard_radiation_mesh_visual_colored_r3.world"
echo "Radiation sources: alpha, beta, gamma, delta"
echo "Motion enabled: $ENABLE_MOTION"
echo "Metrics CSV: $METRICS_CSV"
echo "Logs: $LOG_ROOT"

wait || true
