#!/usr/bin/env bash
set -eo pipefail

WS="/home/i/terrain_radiation_ws"
SETUP="$WS/install/setup.bash"

STAMP=$(date +%Y%m%d_%H%M%S)
RESULT_DIR="$WS/results/aco_evaluation/$STAMP"
ACO_CSV="$RESULT_DIR/aco_hard.csv"

LAUNCH_DIR="$WS/install/radiation_mapping/share/radiation_mapping/launch"

GAZEBO_LAUNCH="$LAUNCH_DIR/gazebo_radiation_plugin.launch.py"
TERRAIN_LAUNCH="$LAUNCH_DIR/terrain_services.launch.py"
ACO_LAUNCH="$LAUNCH_DIR/final_aco.launch.py"
EVALUATOR_LAUNCH="$LAUNCH_DIR/path_terrain_evaluator.launch.py"

mkdir -p "$RESULT_DIR"

source "$SETUP"

open_terminal() {
    local title="$1"
    local command="$2"

    gnome-terminal \
        --title="$title" \
        -- bash -lc "
            source '$SETUP'
            $command
            echo
            echo 'Process stopped. Press Enter to close.'
            read
        "
}

wait_for_topic() {
    local topic="$1"

    for _ in $(seq 1 90); do
        if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
            echo "READY: $topic"
            return 0
        fi

        sleep 1
    done

    echo "TIMEOUT: $topic"
    return 1
}

wait_for_node() {
    local node="$1"

    for _ in $(seq 1 90); do
        if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
            echo "READY: $node"
            return 0
        fi

        sleep 1
    done

    echo "TIMEOUT: $node"
    return 1
}

echo "Result directory:"
echo "$RESULT_DIR"
echo

open_terminal \
    "01 Gazebo Radiation" \
    "ros2 launch '$GAZEBO_LAUNCH'"

sleep 3

open_terminal \
    "02 Hard Terrain" \
    "ros2 launch '$TERRAIN_LAUNCH' terrain:=hard use_sim_time:=true"

sleep 2

open_terminal \
    "03 ACO Planner Smoke Test" \
    "ros2 launch '$ACO_LAUNCH' \
        aco_ant_count:=20 \
        aco_iterations:=20 \
        aco_max_steps:=180 \
        aco_seed:=31 \
        use_sim_time:=true"

sleep 2

open_terminal \
    "04 ACO Path Evaluator" \
    "ros2 launch '$EVALUATOR_LAUNCH' \
        terrain:=hard \
        path_topic:=/aco_path \
        planner_name:=aco \
        radiation_topic:=/radiation_map \
        metrics_topic:=/aco_path_metrics \
        sample_step_m:=0.10 \
        csv_path:='$ACO_CSV' \
        use_sim_time:=true"

wait_for_topic "/terrain_impedance_map"
wait_for_topic "/radiation_map"
wait_for_node "/aco_planner"
wait_for_node "/terrain_path_evaluator"

echo
echo "===== STARTUP CHECK ====="
ros2 node list

echo
echo "ACO parameters:"
ros2 param get /aco_planner aco_ant_count
ros2 param get /aco_planner aco_iterations
ros2 param get /aco_planner include_time_penalty

echo
echo "ACO CSV:"
echo "$ACO_CSV"

echo
echo "All required ACO smoke-test processes have been opened."
