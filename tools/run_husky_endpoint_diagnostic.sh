#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"

DURATION="${1:-15}"
python3 "${HOME}/terrain_radiation_ws/tools/husky_endpoint_diagnostic.py" \
  --duration "${DURATION}" \
  --goal-x -1.13 \
  --goal-y -7.80 \
  --world "${HOME}/terrain_radiation_ws/src/radiation_mapping/worlds/module36_hard_radiation_plugin.world" \
  --heightmap "${HOME}/terrain_radiation_ws/src/radiation_mapping/dem/processed/dem_terrain_hard_husky_015_513.png" \
  --odom-topic /ground_truth/odom \
  --contact-topic /gazebo/dem_inspired_benchmark_world/physics/contacts \
  --model-name husky \
  --base-link base_link
