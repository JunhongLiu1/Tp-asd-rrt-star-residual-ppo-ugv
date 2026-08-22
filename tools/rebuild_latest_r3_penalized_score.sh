#!/usr/bin/env bash
set -eo pipefail

BASE="${HOME}/terrain_radiation_ws/final_score_calibration_r3"
EVALUATOR="${HOME}/terrain_radiation_ws/tools/build_penalized_final_score_v5_1.py"
LATEST="$(
    find "${BASE}" -name run_manifest.tsv -printf '%T@ %h\n' 2>/dev/null \
        | sort -n | tail -n 1 | cut -d' ' -f2-
)"

if [ -z "${LATEST}" ]; then
    echo "[ERROR] No R3 run_manifest.tsv found under ${BASE}" >&2
    exit 1
fi

ARGS=(
    --manifest "${LATEST}/run_manifest.tsv"
    --output-dir "${LATEST}"
)
if [ -f "${LATEST}/radiation_map_stats.json" ]; then
    ARGS+=(--radiation-map-stats "${LATEST}/radiation_map_stats.json")
fi

python3 "${EVALUATOR}" "${ARGS[@]}"

echo
echo "========== PENALIZED FINAL SCORE CSV =========="
column -s, -t "${LATEST}/penalized_final_score_comparison.csv"
