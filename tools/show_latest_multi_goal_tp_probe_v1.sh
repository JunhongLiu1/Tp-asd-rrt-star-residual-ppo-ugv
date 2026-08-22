#!/usr/bin/env bash
set -eo pipefail

LATEST="$(
  find "${HOME}/terrain_radiation_ws/multi_goal_tp_probe" \
    -mindepth 1 -maxdepth 1 -type d \
    -name 'multi_goal_tp_probe_s*' \
    -printf '%T@ %p\n' 2>/dev/null |
  sort -n |
  tail -n 1 |
  cut -d' ' -f2-
)"

if [ -z "${LATEST}" ]; then
    echo "[ERROR] No multi-goal TP probe output was found"
    exit 1
fi

echo "LATEST=${LATEST}"

echo
echo "========== TP CONDITION REPORT =========="
column -s, -t "${LATEST}/tp_condition_report.csv"

echo
echo "========== SCENARIO RANKINGS =========="
column -s, -t "${LATEST}/multi_goal_scenario_ranking.csv"

echo
echo "========== ALL RUNS =========="
column -s, -t "${LATEST}/multi_goal_all_runs.csv"
