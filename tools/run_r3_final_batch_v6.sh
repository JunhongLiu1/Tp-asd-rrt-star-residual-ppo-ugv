#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

WS="${HOME}/terrain_radiation_ws"
RUNNER="${WS}/tools/run_formal_experiment_v3.sh"
AGGREGATOR="${WS}/tools/aggregate_r3_final_batch_v6.py"
CONFIG="${WS}/config/final_score_r3_frozen_v1.json"

SCENARIO="H_S1_R3_final"
REPEATS="${1:-10}"
SEED_ARGUMENT=31
STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH_DIR="${WS}/final_score_formal_r3/formal_${SCENARIO}_n${REPEATS}_${STAMP}"
MANIFEST="${BATCH_DIR}/run_manifest.tsv"

if ! [[ "${REPEATS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] Repeats must be a positive integer"
    exit 2
fi

for required in "${RUNNER}" "${AGGREGATOR}" "${CONFIG}"; do
    if [ ! -e "${required}" ]; then
        echo "[ERROR] Missing required file: ${required}"
        exit 1
    fi
done

mkdir -p "${BATCH_DIR}"
printf "repeat_index\tplanner_key\tseed_argument\trunner_exit_code\tsummary_json\n" > "${MANIFEST}"

echo "======================================================================"
echo "R3 FORMAL FINAL-SCORE BATCH"
echo "scenario = ${SCENARIO}"
echo "repeats per planner = ${REPEATS}"
echo "total attempts = $((REPEATS * 3))"
echo "seed argument = ${SEED_ARGUMENT}"
echo "batch_dir = ${BATCH_DIR}"
echo "======================================================================"
echo "The seed argument remains 31 because the current ASD/TP planners use"
echo "their verified fixed/default seed. These are repeated execution trials,"
echo "not independent planning-seed trials."
echo "======================================================================"

for repeat in $(seq 1 "${REPEATS}"); do
    repeat_tag="$(printf '%02d' "${repeat}")"

    for planner in asd tp aco; do
        run_id="formal_r3_${STAMP}_${planner}_r${repeat_tag}"

        echo
        echo "======================================================================"
        echo "FORMAL ATTEMPT: repeat=${repeat}/${REPEATS} planner=${planner}"
        echo "run_id=${run_id}"
        echo "======================================================================"

        set +e
        "${RUNNER}" \
            --planner "${planner}" \
            --scenario "${SCENARIO}" \
            --run-id "${run_id}" \
            --seed "${SEED_ARGUMENT}" \
            --stack-timeout 180 \
            --planner-timeout 180
        exit_code=$?
        set -e

        planner_dir="${WS}/formal_experiments_v3/${SCENARIO}/${planner}"
        summary="$(
            find "${planner_dir}" -maxdepth 2 -type f \
                -path "*/run_${run_id}_*/summary.json" \
                -printf '%T@ %p\n' 2>/dev/null |
            sort -n |
            tail -n 1 |
            cut -d' ' -f2-
        )"

        if [ -z "${summary}" ]; then
            echo "[WARN] summary.json was not found for ${run_id}"
        else
            echo "[INFO] summary=${summary}"
        fi

        printf "%s\t%s\t%s\t%s\t%s\n" \
            "${repeat}" "${planner}" "${SEED_ARGUMENT}" \
            "${exit_code}" "${summary}" >> "${MANIFEST}"
    done
done

python3 "${AGGREGATOR}" \
    --manifest "${MANIFEST}" \
    --output-dir "${BATCH_DIR}" \
    --config "${CONFIG}"

echo
echo "======================================================================"
echo "FORMAL BATCH COMPLETE"
echo "======================================================================"
echo "Output: ${BATCH_DIR}"
