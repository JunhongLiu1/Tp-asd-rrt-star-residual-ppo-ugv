#!/usr/bin/env bash
set -eo pipefail

BASE="${HOME}/terrain_radiation_ws/final_score_calibration_r3"
LATEST="$(
    find "${BASE}" -name normalized_final_score_comparison.csv \
        -printf '%T@ %h\n' 2>/dev/null \
        | sort -n | tail -n 1 | cut -d' ' -f2-
)"

if [ -z "${LATEST}" ]; then
    echo "[ERROR] No R3 pilot output found under ${BASE}" >&2
    exit 1
fi

echo "Output directory: ${LATEST}"
echo
echo "========== FINAL SCORE CSV =========="
column -s, -t "${LATEST}/normalized_final_score_comparison.csv"
echo
echo "========== R3 PILOT ASSESSMENT =========="
python3 -m json.tool "${LATEST}/r3_pilot_assessment.json"
