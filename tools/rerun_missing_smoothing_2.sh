#!/usr/bin/env bash
set +e

WS="$HOME/terrain_radiation_ws"
PLANNER="$WS/src/radiation_mapping/radiation_mapping/final_asd_rrt_star_planner.py"
COST="$WS/src/radiation_mapping/config/final_cost_model_v1.json"
BACKUP="${PLANNER}.before_missing_smoothing_retry_$(date +%Y%m%d_%H%M%S)"

source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
cd "$WS" || exit 1

cp "$PLANNER" "$BACKUP"

cleanup()
{
    pkill -f formal_experiment_runner_v3.py 2>/dev/null || true
    pkill -f formal_path_waypoint_follower 2>/dev/null || true
    pkill -f spawn_entity 2>/dev/null || true
    pkill -f gazebo 2>/dev/null || true
    pkill -TERM -x gzserver 2>/dev/null || true
    pkill -TERM -x gzclient 2>/dev/null || true
    pkill -f robot_state_publisher 2>/dev/null || true
    pkill -f controller_manager 2>/dev/null || true
    sleep 8
    pkill -KILL -x gzserver 2>/dev/null || true
    pkill -KILL -x gzclient 2>/dev/null || true
    sleep 12
}

set_mode()
{
    mode="$1"

    python3 - "$PLANNER" "$mode" <<'PY'
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
mode = sys.argv[2]

settings = {
    "shortcut": (True, False),
    "teb": (True, True),
}

a, b = settings[mode]
text = path.read_text()

text, n1 = re.subn(
    r"self\.enable_path_smoothing\s*=\s*(True|False)",
    f"self.enable_path_smoothing = {a}",
    text,
    count=1,
)

text, n2 = re.subn(
    r"self\.enable_teb_smoothing\s*=\s*(True|False)",
    f"self.enable_teb_smoothing = {b}",
    text,
    count=1,
)

if n1 != 1 or n2 != 1:
    raise SystemExit(f"replacement failed: path={n1}, teb={n2}")

path.write_text(text)
print(f"[MODE] {mode}: path={a}, teb={b}")
PY
}

build()
{
    source /opt/ros/foxy/setup.bash
    colcon build \
      --packages-select radiation_mapping \
      --symlink-install
    source "$WS/install/setup.bash"
}

run_one()
{
    scenario="$1"
    mode="$2"
    seed="$3"

    echo "=================================================="
    echo "scenario=$scenario mode=$mode seed=$seed"
    echo "=================================================="

    cp "$BACKUP" "$PLANNER"
    set_mode "$mode"
    build || return 1
    cleanup

    bash tools/run_formal_experiment_v3.sh \
      --planner asd \
      --scenario "$scenario" \
      --run-id "smoothing_${mode}_${scenario}_s${seed}_retry" \
      --seed "$seed" \
      --cost-model-config "$COST" \
      --stack-timeout 360 \
      --planner-timeout 300

    code=$?
    echo "FINISHED scenario=$scenario mode=$mode seed=$seed exit_code=$code"
    sleep 15
}

run_one H_S1_R3_final shortcut 31
run_one H_T1_terrain teb 111

cp "$BACKUP" "$PLANNER"
build

echo "Original planner restored."
