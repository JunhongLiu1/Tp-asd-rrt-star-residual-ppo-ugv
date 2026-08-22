#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
"${HOME}/terrain_radiation_ws/tools/run_formal_experiment_v3.sh" \
  --planner aco \
  --scenario H_S1_R3_final \
  --run-id "aco_vehicle_safe_v7_s31_${STAMP}" \
  --seed 31 \
  --stack-timeout 180 \
  --planner-timeout 300
