#!/usr/bin/env bash
set +u
set -o pipefail
WS="${HOME}/terrain_radiation_ws"
cd "$WS" || { echo "ERROR: workspace not found: $WS"; exit 10; }
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
exec python3 "$WS/tools/formal_experiment_runner_v3.py" "$@"
