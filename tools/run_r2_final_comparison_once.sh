#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

WS="${HOME}/terrain_radiation_ws"
RUNNER="${WS}/tools/run_formal_experiment_v3.sh"
EVALUATOR="${WS}/tools/build_normalized_final_score_v4.py"
REFERENCES="${WS}/config/final_score_references_r2_frozen.json"
SCENARIO="${1:-H_S1_R2_final}"
SEED="${2:-32}"
STAMP="$(date +%Y%m%d_%H%M%S)"
COMPARISON_ID="r2_final_once_${SCENARIO}_s${SEED}_${STAMP}"
OUT="${WS}/final_score_comparisons_r2/${COMPARISON_ID}"
MANIFEST="${OUT}/run_manifest.tsv"

if [ ! -f "${REFERENCES}" ]; then
    echo "[ERROR] Frozen references not found: ${REFERENCES}" >&2
    echo "Run freeze_r2_normalization_references.sh first." >&2
    exit 1
fi

mkdir -p "${OUT}"
printf 'scenario\tplanner_key\tseed\trunner_exit_code\tsummary_json\n' > "${MANIFEST}"

find_summary() {
    local planner="$1"
    local run_id="$2"
    local base="${WS}/formal_experiments_v3/${SCENARIO}/${planner}"
    find "${base}" -mindepth 2 -maxdepth 2 -type f \
        -path "*/run_${run_id}_*/summary.json" \
        -printf '%T@\t%p\n' 2>/dev/null \
        | sort -n | tail -n 1 | cut -f2-
}

for planner in asd tp aco; do
    run_id="${COMPARISON_ID}_${planner}"
    set +e
    "${RUNNER}" \
        --planner "${planner}" \
        --scenario "${SCENARIO}" \
        --run-id "${run_id}" \
        --seed "${SEED}" \
        --stack-timeout 180 \
        --planner-timeout 180
    code=$?
    set -e
    sleep 4
    summary="$(find_summary "${planner}" "${run_id}")"
    printf '%s\t%s\t%s\t%s\t%s\n' \
        "${SCENARIO}" "${planner}" "${SEED}" "${code}" "${summary}" \
        >> "${MANIFEST}"
done

python3 "${EVALUATOR}" \
    --manifest "${MANIFEST}" \
    --output-dir "${OUT}" \
    --mode evaluate \
    --references "${REFERENCES}"
