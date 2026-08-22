#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

WS="${HOME}/terrain_radiation_ws"
PKG="${WS}/src/radiation_mapping"
TOOLS="${WS}/tools"
RUNNER="${TOOLS}/run_formal_experiment_v3.sh"
AGGREGATOR="${TOOLS}/build_multi_goal_tp_probe_v1.py"
SCENARIO_CONFIG="${PKG}/config/formal_scenarios_v1.json"

SEED="${1:-31}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${WS}/multi_goal_tp_probe/multi_goal_tp_probe_s${SEED}_${STAMP}"
MANIFEST="${OUT}/run_manifest.tsv"

SCENARIOS=(
  H_MG1_short_east
  H_MG2_long_south
)

PLANNERS=(
  asd
  tp
  aco
)

for required in "${RUNNER}" "${AGGREGATOR}" "${SCENARIO_CONFIG}"; do
    if [ ! -e "${required}" ]; then
        echo "[ERROR] Missing required file: ${required}" >&2
        exit 1
    fi
done

mkdir -p "${OUT}"
printf "scenario\tplanner_key\tseed\trunner_exit_code\tsummary_json\n" > "${MANIFEST}"

TOTAL=$(( ${#SCENARIOS[@]} * ${#PLANNERS[@]} ))
COMPLETED=0

echo "======================================================================"
echo "MULTI-GOAL TP PROBE"
echo "Three fixed goals x three planners x one run = ${TOTAL} attempts"
echo "Seed argument: ${SEED}"
echo "Output: ${OUT}"
echo "======================================================================"

for scenario in "${SCENARIOS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        attempt=$((COMPLETED + 1))
        run_id="multi_goal_tp_probe_${STAMP}_${scenario}_${planner}"

        echo
        echo "======================================================================"
        echo "ATTEMPT ${attempt}/${TOTAL}"
        echo "scenario=${scenario}"
        echo "planner=${planner}"
        echo "run_id=${run_id}"
        echo "======================================================================"

        set +e
        "${RUNNER}" \
            --planner "${planner}" \
            --scenario "${scenario}" \
            --run-id "${run_id}" \
            --seed "${SEED}" \
            --stack-timeout 180 \
            --planner-timeout 180
        exit_code=$?
        set -e

        planner_dir="${WS}/formal_experiments_v3/${scenario}/${planner}"
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
            "${scenario}" "${planner}" "${SEED}" \
            "${exit_code}" "${summary}" >> "${MANIFEST}"

        COMPLETED=$((COMPLETED + 1))
        echo "[PROGRESS] completed ${COMPLETED}/${TOTAL}"
    done
done

python3 "${AGGREGATOR}" \
    --manifest "${MANIFEST}" \
    --scenario-config "${SCENARIO_CONFIG}" \
    --output-dir "${OUT}"

echo
echo "======================================================================"
echo "MULTI-GOAL TP PROBE COMPLETE"
echo "======================================================================"
echo "Output: ${OUT}"
