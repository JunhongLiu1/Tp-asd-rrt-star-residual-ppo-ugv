#!/usr/bin/env bash
set -eo pipefail

WS="/home/i/terrain_radiation_ws"
SETUP="$WS/install/setup.bash"

LAUNCH_DIR="$WS/install/radiation_mapping/share/radiation_mapping/launch"

GAZEBO_LAUNCH="$LAUNCH_DIR/gazebo_radiation_plugin.launch.py"
TERRAIN_LAUNCH="$LAUNCH_DIR/terrain_services.launch.py"
ACO_LAUNCH="$LAUNCH_DIR/final_aco.launch.py"
EVALUATOR_LAUNCH="$LAUNCH_DIR/path_terrain_evaluator.launch.py"

STAMP=$(date +%Y%m%d_%H%M%S)
RESULT_DIR="$WS/results/week7_report/$STAMP"

ACO_CSV="$RESULT_DIR/aco_50x70_hard.csv"
STATUS_FILE="$RESULT_DIR/ros2_system_status.txt"

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

wait_for_node() {
    local node_name="$1"

    for _ in $(seq 1 120); do
        if ros2 node list 2>/dev/null \
            | grep -Fxq "$node_name"; then
            echo "READY NODE: $node_name"
            return 0
        fi

        sleep 1
    done

    echo "TIMEOUT NODE: $node_name"
    return 1
}

wait_for_topic() {
    local topic_name="$1"

    for _ in $(seq 1 120); do
        if ros2 topic list 2>/dev/null \
            | grep -Fxq "$topic_name"; then
            echo "READY TOPIC: $topic_name"
            return 0
        fi

        sleep 1
    done

    echo "TIMEOUT TOPIC: $topic_name"
    return 1
}

echo "Refreshing ROS 2 discovery..."
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 2

EXISTING_NODES=$(
    ros2 node list 2>/dev/null \
        | grep -E \
        '^/aco_planner$|^/gazebo$|^/radiation_world_plugin$|^/terrain_layer_publisher$|^/terrain_path_evaluator$' \
        || true
)

if [ -n "$EXISTING_NODES" ]; then
    echo
    echo "ERROR: old experiment nodes are still running:"
    echo "$EXISTING_NODES"
    echo
    echo "Close the old experiment terminals before running this script."
    exit 1
fi

echo
echo "Result directory:"
echo "$RESULT_DIR"
echo

open_terminal \
    "01 Gazebo Radiation Simulation" \
    "ros2 launch '$GAZEBO_LAUNCH'"

sleep 3

open_terminal \
    "02 Hard Terrain Publisher" \
    "ros2 launch '$TERRAIN_LAUNCH' \
        terrain:=hard \
        use_sim_time:=true"

sleep 2

open_terminal \
    "03 Formal ACO 50x70" \
    "ros2 launch '$ACO_LAUNCH' \
        aco_ant_count:=50 \
        aco_iterations:=70 \
        aco_max_steps:=240 \
        aco_seed:=31 \
        include_time_penalty:=false \
        use_sim_time:=true"

sleep 2

open_terminal \
    "04 Formal ACO Evaluator" \
    "ros2 launch '$EVALUATOR_LAUNCH' \
        terrain:=hard \
        path_topic:=/aco_path \
        planner_name:=aco_50x70 \
        radiation_topic:=/radiation_map \
        metrics_topic:=/aco_50x70_path_metrics \
        sample_step_m:=0.10 \
        csv_path:='$ACO_CSV' \
        use_sim_time:=true"

echo
echo "Waiting for the formal experiment pipeline..."

wait_for_node "/radiation_world_plugin"
wait_for_node "/terrain_layer_publisher"
wait_for_node "/aco_planner"
wait_for_node "/terrain_path_evaluator"

wait_for_topic "/radiation_map"
wait_for_topic "/terrain_impedance_map"
wait_for_topic "/odom"
wait_for_topic "/goal_pose"
wait_for_topic "/aco_path"

sleep 3

{
    echo "============================================================"
    echo "       WEEK 7 FORMAL ROS 2 ACO EXPERIMENT STATUS"
    echo "============================================================"

    echo
    echo "===== REQUIRED ROS 2 NODES ====="

    ros2 node list \
        | grep -E \
        '^/aco_planner$|^/radiation_world_plugin$|^/terrain_layer_publisher$|^/terrain_path_evaluator$'

    echo
    echo "===== REQUIRED ROS 2 TOPICS ====="

    ros2 topic list \
        | grep -E \
        '^/aco_path$|^/radiation_map$|^/terrain_impedance_map$|^/goal_pose$|^/odom$'

    echo
    echo "===== FORMAL ACO PARAMETERS ====="

    echo -n "Ant count: "
    ros2 param get /aco_planner aco_ant_count

    echo -n "Iterations: "
    ros2 param get /aco_planner aco_iterations

    echo -n "Maximum steps: "
    ros2 param get /aco_planner aco_max_steps

    echo -n "Random seed: "
    ros2 param get /aco_planner aco_seed

    echo -n "Time penalty enabled: "
    ros2 param get /aco_planner include_time_penalty

    echo -n "Non-traversable threshold: "
    ros2 param get \
        /aco_planner \
        aco_nontraversable_threshold

    echo
    echo "===== ACO PATH CONNECTION ====="

    ros2 topic info /aco_path \
        | grep -E \
        'Publisher count|Subscription count'

    echo
    echo "===== RADIATION MAP CONNECTION ====="

    ros2 topic info /radiation_map \
        | grep -E \
        'Publisher count|Subscription count'

    echo
    echo "===== TERRAIN MAP CONNECTION ====="

    ros2 topic info /terrain_impedance_map \
        | grep -E \
        'Publisher count|Subscription count'

    echo
    echo "===== FORMAL TEST CONFIGURATION ====="

    echo "Terrain: hard"
    echo "Start: obtained from /odom"
    echo "Goal: (0.0, -12.0)"
    echo "Planner: ACO 50x70"
    echo "Cost profile: balanced"
    echo "Time penalty: disabled"
    echo "CSV: $ACO_CSV"

    echo
    echo "============================================================"
} | tee "$STATUS_FILE"

echo
echo "The ROS 2 status above is ready for the report screenshot."
echo
read -rp "Press Enter to send the formal goal (0, -12)..."

ros2 topic pub --once \
    /goal_pose \
    geometry_msgs/msg/PoseStamped \
    "{
        header: {
            frame_id: 'map'
        },
        pose: {
            position: {
                x: 0.0,
                y: -12.0,
                z: 0.0
            },
            orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        }
    }"

echo
echo "Formal goal sent."
echo "Watch terminal 03 for ACO convergence logs."
echo "Waiting for the evaluator result..."

RESULT_READY=false

for _ in $(seq 1 360); do
    if [ -f "$ACO_CSV" ]; then
        LINE_COUNT=$(wc -l < "$ACO_CSV")

        if [ "$LINE_COUNT" -ge 2 ]; then
            RESULT_READY=true
            break
        fi
    fi

    sleep 1
done

echo

if [ "$RESULT_READY" = true ]; then
    echo "============================================================"
    echo "              FORMAL ACO EVALUATION RESULT"
    echo "============================================================"

    tail -n 1 "$ACO_CSV"

    echo
    echo "CSV saved to:"
    echo "$ACO_CSV"
else
    echo "No evaluator row was written within 360 seconds."
    echo "Check terminal 03 and terminal 04 for errors."
fi

echo
echo "ROS 2 status saved to:"
echo "$STATUS_FILE"

echo
echo "Keep the four process terminals open while taking screenshots."
