#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/foxy/setup.bash
source /home/i/terrain_radiation_ws/install/setup.bash
set -u

workspace=/home/i/terrain_radiation_ws
output_root=${1:-$workspace/acceptance_logs/residual_gazebo_matrix_20260822}
checkpoint=$workspace/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/residual_ppo_final.zip
manifest=$workspace/acceptance_logs/rl_ppo_20260822/retrain50k_checkpoints/training_manifest.json
checkpoint_sha=8233e2504909a97844cb3f97c72ab7c7756b1762a997a48168904256c2f1c742
default_scenarios="lateral_offset heading_offset turn_90 s_curve medium_path estop_replan_recover"
read -r -a scenarios <<< "${SCENARIOS:-$default_scenarios}"
read -r -a policies <<< "${POLICIES:-zero ppo}"

mkdir -p "$output_root/runtime"
export ROS_LOG_DIR="$output_root/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd /tmp

gazebo_pid=
terrain_pid=
stack_pid=

stop_group() {
  local pid=${1:-}
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      sleep 1
    fi
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || true
    fi
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  stop_group "$stack_pid"
  stop_group "$terrain_pid"
  stop_group "$gazebo_pid"
}
on_signal() {
  cleanup
  trap - EXIT
  exit 130
}
trap cleanup EXIT
trap on_signal INT TERM

setsid ros2 launch radiation_mapping gazebo_radiation_husky_formal.launch.py \
  gui:=false start_x:=5.134 start_y:=5.977 start_z:=0.448 start_yaw:=0.0 \
  >"$output_root/runtime/gazebo.log" 2>&1 &
gazebo_pid=$!
setsid ros2 launch radiation_mapping terrain_services.launch.py \
  terrain:=hard use_sim_time:=true >"$output_root/runtime/terrain.log" 2>&1 &
terrain_pid=$!

timeout 90 bash -c \
  'until ros2 service list 2>/dev/null | grep -qx /set_entity_state && ros2 topic list 2>/dev/null | grep -qx /odometry/filtered; do sleep 1; done'

for policy in "${policies[@]}"; do
  policy_type=zero
  if [[ "$policy" == ppo ]]; then
    policy_type=sb3
  fi
  mkdir -p "$output_root/$policy"
  setsid ros2 launch risk_aware_planner_cpp \
    tp_asd_rrt_star_online_radiation.launch.py \
    enable_motion:=true enable_frame_alignment:=false \
    enable_velocity_pid:=true enable_residual_rl:=true \
    residual_policy_type:="$policy_type" \
    residual_checkpoint_path:="$checkpoint" \
    residual_checkpoint_manifest_path:="$manifest" \
    residual_checkpoint_sha256_allowlist:="$checkpoint_sha" \
    residual_worker_python_executable:=/usr/bin/python3 \
    residual_worker_pythonpath:="$workspace/.rl_deps" \
    metrics_csv:="$output_root/$policy/planner_metrics.csv" \
    >"$output_root/runtime/stack_${policy}.log" 2>&1 &
  stack_pid=$!
  timeout 45 bash -c \
    'until ros2 topic list 2>/dev/null | grep -qx /control/residual_rl_status && ros2 topic list 2>/dev/null | grep -qx /cmd_vel; do sleep 1; done'
  sleep 4

  run=1
  for scenario in "${scenarios[@]}"; do
    trial_dir="$output_root/$policy/$scenario"
    mkdir -p "$trial_dir"
    rm -f "$trial_dir/result.json" "$trial_dir/exit_code.txt"
    set +e
    /usr/bin/python3 "$workspace/tools/pi_gazebo_trial.py" \
      --scenario "$scenario" --pid on --policy "$policy" --run "$run" \
      --path-frame map \
      --reset-x 5.134 --reset-y 5.977 --reset-z 0.448 \
      --output "$trial_dir" --timeout-sec 75 \
      >"$trial_dir/trial.log" 2>&1
    trial_rc=$?
    set -e
    printf '%s\n' "$trial_rc" >"$trial_dir/exit_code.txt"
    run=$((run + 1))
    sleep 2
  done
  stop_group "$stack_pid"
  stack_pid=
  sleep 3
done
