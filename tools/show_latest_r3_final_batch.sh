#!/usr/bin/env bash
set -eo pipefail

LATEST="$(
  find "${HOME}/terrain_radiation_ws/final_score_formal_r3" \
    -mindepth 1 -maxdepth 1 -type d \
    -name 'formal_H_S1_R3_final_n*' \
    -printf '%T@ %p\n' 2>/dev/null |
  sort -n |
  tail -n 1 |
  cut -d' ' -f2-
)"

if [ -z "${LATEST}" ]; then
    echo "[ERROR] No R3 formal batch directory found"
    exit 1
fi

echo "LATEST=${LATEST}"
echo
echo "========== PLANNER SUMMARY =========="
column -s, -t "${LATEST}/final_score_planner_summary.csv"
echo
echo "========== PER-RUN RESULTS =========="
column -s, -t "${LATEST}/final_score_formal_runs.csv"
