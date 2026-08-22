#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

WS="${HOME}/terrain_radiation_ws"
RUNNER="${WS}/tools/run_formal_experiment_v3.sh"
EVALUATOR="${WS}/tools/build_normalized_final_score_v5.py"
MAP_CAPTURE="${WS}/tools/capture_radiation_map_stats_v5.py"
SCENARIO="${1:-H_S1_R3_final}"
SEED="${2:-31}"
STAMP="$(date +%Y%m%d_%H%M%S)"
PILOT_ID="r3_pilot_${SCENARIO}_s${SEED}_${STAMP}"
OUT="${WS}/final_score_calibration_r3/${PILOT_ID}"
MANIFEST="${OUT}/run_manifest.tsv"
MAP_STATS="${OUT}/radiation_map_stats.json"

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

run_one() {
    local planner="$1"
    local capture_map="$2"
    local run_id="${PILOT_ID}_${planner}"
    local code=0

    echo
    echo "========================================================================"
    echo "R3 PILOT: scenario=${SCENARIO} planner=${planner} seed=${SEED}"
    echo "========================================================================"

    if [ "${capture_map}" = "yes" ]; then
        set +e
        "${RUNNER}" \
            --planner "${planner}" \
            --scenario "${SCENARIO}" \
            --run-id "${run_id}" \
            --seed "${SEED}" \
            --stack-timeout 180 \
            --planner-timeout 180 &
        local runner_pid=$!

        python3 "${MAP_CAPTURE}" \
            --topic /radiation_map \
            --output "${MAP_STATS}" \
            --timeout 180
        local capture_code=$?
        wait "${runner_pid}"
        code=$?
        set -e

        if [ "${capture_code}" -ne 0 ]; then
            echo "[WARN] Radiation-map capture failed with code ${capture_code}"
        fi
    else
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
    fi

    sleep 4
    local summary=""
    summary="$(find_summary "${planner}" "${run_id}")"
    printf '%s\t%s\t%s\t%s\t%s\n' \
        "${SCENARIO}" "${planner}" "${SEED}" "${code}" "${summary}" \
        >> "${MANIFEST}"
}

run_one asd yes
run_one tp no
run_one aco no

python3 "${EVALUATOR}" \
    --manifest "${MANIFEST}" \
    --output-dir "${OUT}" \
    --radiation-map-stats "${MAP_STATS}"

echo
echo "R3 pilot complete. Candidate references remain UNFROZEN."
echo "Output: ${OUT}"
