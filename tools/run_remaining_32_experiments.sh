#!/usr/bin/env bash

set +e

WS="$HOME/terrain_radiation_ws"
PLANNER_FILE="$WS/src/radiation_mapping/radiation_mapping/final_asd_rrt_star_planner.py"
COST_MODEL="$WS/src/radiation_mapping/config/final_cost_model_v1.json"
BACKUP_FILE="${PLANNER_FILE}.before_smoothing_ablation_$(date +%Y%m%d_%H%M%S)"

SEEDS=(31 51 71 91 111)
SCENARIOS=(H_S1_R3_final H_T1_terrain)
MODES=(raw shortcut teb)

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"

cd "$WS" || exit 1

cp "$PLANNER_FILE" "$BACKUP_FILE"

echo "Planner backup: $BACKUP_FILE"

restore_source()
{
    if [ -f "$BACKUP_FILE" ]; then
        cp "$BACKUP_FILE" "$PLANNER_FILE"
        echo "[RESTORE] Original planner source restored."
    fi
}

trap restore_source EXIT INT TERM

clean_stack()
{
    pkill -f formal_experiment_runner_v3.py 2>/dev/null || true
    pkill -f formal_path_waypoint_follower 2>/dev/null || true
    pkill -f spawn_entity 2>/dev/null || true
    pkill -f gazebo 2>/dev/null || true
    pkill -TERM -x gzserver 2>/dev/null || true
    pkill -TERM -x gzclient 2>/dev/null || true
    pkill -f robot_state_publisher 2>/dev/null || true
    pkill -f controller_manager 2>/dev/null || true

    sleep 7

    pkill -KILL -x gzserver 2>/dev/null || true
    pkill -KILL -x gzclient 2>/dev/null || true

    sleep 13
}

build_package()
{
    echo "===== BUILD radiation_mapping ====="

    cd "$WS" || return 1

    source /opt/ros/foxy/setup.bash

    colcon build \
        --packages-select radiation_mapping \
        --symlink-install

    local code=$?

    if [ "$code" -ne 0 ]; then
        echo "[ERROR] Build failed with exit code $code"
        return "$code"
    fi

    source "$WS/install/setup.bash"
    return 0
}

set_smoothing_mode()
{
    local mode="$1"

    python3 - "$PLANNER_FILE" "$mode" <<'PY'
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
mode = sys.argv[2]

settings = {
    "raw": (False, False),
    "shortcut": (True, False),
    "teb": (True, True),
}

if mode not in settings:
    raise SystemExit(f"Unknown smoothing mode: {mode}")

enable_path, enable_teb = settings[mode]

text = path.read_text(encoding="utf-8")

text, path_count = re.subn(
    r"self\.enable_path_smoothing\s*=\s*(?:True|False)",
    f"self.enable_path_smoothing = {enable_path}",
    text,
    count=1,
)

text, teb_count = re.subn(
    r"self\.enable_teb_smoothing\s*=\s*(?:True|False)",
    f"self.enable_teb_smoothing = {enable_teb}",
    text,
    count=1,
)

if path_count != 1:
    raise SystemExit(
        f"Expected one enable_path_smoothing assignment, found {path_count}"
    )

if teb_count != 1:
    raise SystemExit(
        f"Expected one enable_teb_smoothing assignment, found {teb_count}"
    )

path.write_text(text, encoding="utf-8")

print(
    f"[MODE] {mode}: "
    f"enable_path_smoothing={enable_path}, "
    f"enable_teb_smoothing={enable_teb}"
)
PY
}

run_experiment()
{
    local planner="$1"
    local scenario="$2"
    local seed="$3"
    local run_id="$4"
    local planner_timeout="$5"

    echo
    echo "================================================================"
    echo "NOW RUNNING"
    echo "planner  = $planner"
    echo "scenario = $scenario"
    echo "seed     = $seed"
    echo "run_id   = $run_id"
    echo "================================================================"

    clean_stack

    cd "$WS" || return 1
    source /opt/ros/foxy/setup.bash
    source "$WS/install/setup.bash"

    bash tools/run_formal_experiment_v3.sh \
        --planner "$planner" \
        --scenario "$scenario" \
        --run-id "$run_id" \
        --seed "$seed" \
        --cost-model-config "$COST_MODEL" \
        --stack-timeout 360 \
        --planner-timeout "$planner_timeout"

    local code=$?

    echo "FINISHED planner=$planner scenario=$scenario seed=$seed exit_code=$code"
    sleep 15

    return "$code"
}

mkdir -p "$WS/thesis_results/chapter6/remaining_experiments"

MASTER_LOG="$WS/thesis_results/chapter6/remaining_experiments/run_32_master_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "$MASTER_LOG") 2>&1

echo "Master log: $MASTER_LOG"
echo "Start time: $(date --iso-8601=seconds)"

# ============================================================
# Part A: two dedicated ACO convergence runs
# Current source is the normal shortcut + TEB configuration.
# Iteration information is stored in each run's planner.log.
# ============================================================

echo
echo "############################################################"
echo "PART A: ACO CONVERGENCE RUNS"
echo "############################################################"

run_experiment \
    aco \
    H_MG2_long_south \
    31 \
    aco_convergence_HMG2_s31 \
    300

run_experiment \
    aco \
    H_T1_terrain \
    31 \
    aco_convergence_HT1_s31 \
    300

# ============================================================
# Part B: smoothing ablation
# ASD-RRT* only, because the purpose is to isolate the effect
# of post-processing while keeping the planner unchanged.
# ============================================================

echo
echo "############################################################"
echo "PART B: SMOOTHING ABLATION"
echo "############################################################"

for mode in "${MODES[@]}"; do

    echo
    echo "============================================================"
    echo "CONFIGURING SMOOTHING MODE: $mode"
    echo "============================================================"

    # Always start from the same original source.
    cp "$BACKUP_FILE" "$PLANNER_FILE"

    set_smoothing_mode "$mode"

    if ! build_package; then
        echo "[ERROR] Cannot continue mode=$mode because build failed."
        continue
    fi

    for scenario in "${SCENARIOS[@]}"; do
        for seed in "${SEEDS[@]}"; do

            run_experiment \
                asd \
                "$scenario" \
                "$seed" \
                "smoothing_${mode}_${scenario}_s${seed}" \
                300

        done
    done
done

# ============================================================
# Restore normal planner and rebuild.
# ============================================================

echo
echo "############################################################"
echo "RESTORING NORMAL PLANNER"
echo "############################################################"

cp "$BACKUP_FILE" "$PLANNER_FILE"

build_package

trap - EXIT INT TERM

echo
echo "============================================================"
echo "ALL REQUESTED EXPERIMENTS FINISHED"
echo "End time: $(date --iso-8601=seconds)"
echo "Master log: $MASTER_LOG"
echo "============================================================"
