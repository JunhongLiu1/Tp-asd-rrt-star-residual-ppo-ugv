#!/usr/bin/env bash

set -eo pipefail

WS="/home/i/terrain_radiation_ws"

BASE_CONFIG="$WS/src/radiation_mapping/config/final_cost_model_v1.json"
TP_CONFIG="$WS/src/radiation_mapping/config/tp_cost_model_lambda_1.json"

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RESULT_DIR="$WS/results/time_penalty_sensitivity/$RUN_ID"

ASD_CSV="$RESULT_DIR/asd_rrt_star_hard.csv"
TP_CSV="$RESULT_DIR/tp_asd_rrt_star_lambda_1_hard.csv"

mkdir -p "$RESULT_DIR"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "ERROR: gnome-terminal not found."
    exit 1
fi

if [[ ! -f "$BASE_CONFIG" ]]; then
    echo "ERROR: Missing base config: $BASE_CONFIG"
    exit 1
fi

if [[ ! -f "$TP_CONFIG" ]]; then
    echo "ERROR: Missing TP config: $TP_CONFIG"
    exit 1
fi

source "$WS/install/setup.bash"

ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

launch_terminal() {
    local title="$1"
    local command="$2"

    gnome-terminal \
        --title="$title" \
        -- bash -lc "
            source '$WS/install/setup.bash'
            echo '===== $title ====='
            $command
            echo
            echo 'Process stopped. This terminal will remain open.'
            exec bash
        " &
}

wait_for_publisher() {
    local topic="$1"
    local attempts=60

    for ((i = 1; i <= attempts; i++)); do
        if ros2 topic info "$topic" 2>/dev/null \
            | grep -Eq 'Publisher count: [1-9]'; then
            echo "READY: $topic"
            return 0
        fi

        sleep 1
    done

    echo "ERROR: No publisher appeared for $topic"
    exit 1
}

wait_for_node() {
    local node="$1"
    local attempts=60

    for ((i = 1; i <= attempts; i++)); do
        if ros2 node list 2>/dev/null \
            | grep -Fxq "$node"; then
            echo "READY: $node"
            return 0
        fi

        sleep 1
    done

    echo "ERROR: Node did not appear: $node"
    exit 1
}

echo "Result directory:"
echo "$RESULT_DIR"
echo

launch_terminal \
    "01 Gazebo Radiation World" \
    "ros2 launch radiation_mapping gazebo_radiation_plugin.launch.py"

launch_terminal \
    "02 Hard Terrain Services" \
    "ros2 launch radiation_mapping terrain_services.launch.py terrain:=hard use_sim_time:=true"

wait_for_publisher "/terrain_impedance_map"
wait_for_publisher "/radiation_map"

launch_terminal \
    "03 ASD-RRT Star" \
    "ros2 launch radiation_mapping final_asd_rrt_star_baseline.launch.py"

launch_terminal \
    "04 TP-ASD-RRT Star Lambda 1" \
    "ros2 launch radiation_mapping final_tp_asd_rrt_star.launch.py cost_model_config:='$TP_CONFIG'"

wait_for_node "/asd_rrt_star_planner"
wait_for_node "/tp_asd_rrt_star_planner"

launch_terminal \
    "05 ASD Path Evaluator" \
    "ros2 run radiation_mapping terrain_path_evaluator \
        --ros-args \
        -r __node:=asd_path_evaluator \
        -p terrain_level:=hard \
        -p path_topic:=/asd_rrt_star_path \
        -p radiation_topic:=/radiation_map \
        -p planner_name:=asd_rrt_star \
        -p metrics_topic:=/asd_rrt_star_path_metrics \
        -p cost_model_config:='$BASE_CONFIG' \
        -p cost_profile:=balanced \
        -p radiation_input_mode:=normalized_occupancy \
        -p radiation_input_max:=100.0 \
        -p sample_step_m:=0.10 \
        -p csv_path:='$ASD_CSV' \
        -p use_sim_time:=true"

launch_terminal \
    "06 TP Path Evaluator Lambda 1" \
    "ros2 run radiation_mapping terrain_path_evaluator \
        --ros-args \
        -r __node:=tp_path_evaluator \
        -p terrain_level:=hard \
        -p path_topic:=/tp_asd_rrt_star_path \
        -p radiation_topic:=/radiation_map \
        -p planner_name:=tp_asd_rrt_star_lambda_1 \
        -p metrics_topic:=/tp_asd_rrt_star_path_metrics \
        -p cost_model_config:='$TP_CONFIG' \
        -p cost_profile:=balanced \
        -p radiation_input_mode:=normalized_occupancy \
        -p radiation_input_max:=100.0 \
        -p sample_step_m:=0.10 \
        -p csv_path:='$TP_CSV' \
        -p use_sim_time:=true"

sleep 3

echo
echo "===== STARTUP CHECK ====="

ros2 node list \
    | sort \
    | grep -E \
        'asd_rrt_star_planner|path_evaluator|terrain_layer_publisher|radiation_world_plugin' \
    || true

echo
echo "ASD parameter:"
ros2 param get \
    /asd_rrt_star_planner \
    include_time_penalty

echo
echo "TP parameter:"
ros2 param get \
    /tp_asd_rrt_star_planner \
    include_time_penalty

echo
echo "TP configuration:"
ros2 param get \
    /tp_asd_rrt_star_planner \
    cost_model_config

echo
echo "ASD CSV:"
echo "$ASD_CSV"

echo
echo "TP CSV:"
echo "$TP_CSV"

echo
echo "All required processes have been opened."
