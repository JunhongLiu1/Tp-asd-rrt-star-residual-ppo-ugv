#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

WS="${HOME}/terrain_radiation_ws"
RUNNER="${WS}/tools/run_formal_experiment_v3.sh"
EVALUATOR="${WS}/tools/build_final_score_csv.py"
SCENARIO="${1:-H_B1_balanced}"
SEED="${2:-31}"
STAMP="$(date +%Y%m%d_%H%M%S)"
COMPARISON_ID="final_once_${SCENARIO}_s${SEED}_${STAMP}"
OUT="${WS}/final_score_comparisons/${COMPARISON_ID}"
MANIFEST="${OUT}/run_manifest.tsv"

if [ ! -x "${RUNNER}" ]; then
    echo "[ERROR] Runner not found or not executable: ${RUNNER}"
    exit 1
fi

if [ ! -x "${EVALUATOR}" ]; then
    echo "[ERROR] Evaluator not found or not executable: ${EVALUATOR}"
    exit 1
fi

mkdir -p "${OUT}"
printf 'scenario\tplanner_key\tseed\trunner_exit_code\tsummary_json\n' > "${MANIFEST}"

find_summary()
{
    local planner="$1"
    local run_id="$2"
    local base="${WS}/formal_experiments_v3/${SCENARIO}/${planner}"

    find "${base}" -mindepth 2 -maxdepth 2 -type f \
        -path "*/run_${run_id}_*/summary.json" \
        -printf '%T@\t%p\n' 2>/dev/null \
        | sort -n \
        | tail -n 1 \
        | cut -f2-
}

run_one()
{
    local planner="$1"
    local run_id="${COMPARISON_ID}_${planner}"

    echo
    echo "======================================================================"
    echo "FINAL ONE-RUN COMPARISON"
    echo "scenario=${SCENARIO} planner=${planner} seed=${SEED}"
    echo "run_id=${run_id}"
    echo "======================================================================"

    set +e
    "${RUNNER}" \
        --planner "${planner}" \
        --scenario "${SCENARIO}" \
        --run-id "${run_id}" \
        --seed "${SEED}" \
        --stack-timeout 180 \
        --planner-timeout 180
    local code=$?
    set -e

    sleep 4

    local summary=""
    summary="$(find_summary "${planner}" "${run_id}")"

    printf '%s\t%s\t%s\t%s\t%s\n' \
        "${SCENARIO}" "${planner}" "${SEED}" "${code}" "${summary}" \
        >> "${MANIFEST}"

    if [ -z "${summary}" ]; then
        echo "[ERROR] summary.json not found for ${planner}"
    else
        echo "[INFO] summary=${summary}"
    fi
}

# Same map, start, goal, radiation profile and seed for all algorithms.
run_one asd
run_one tp
run_one aco

python3 "${EVALUATOR}" \
    --manifest "${MANIFEST}" \
    --output-dir "${OUT}"
