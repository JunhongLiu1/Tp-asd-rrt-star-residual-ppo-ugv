#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash
source "${HOME}/terrain_radiation_ws/install/setup.bash"
set -u

RUNNER="${HOME}/terrain_radiation_ws/tools/run_formal_experiment_v3.sh"
SEED=31
BATCH_TIME="$(date +%Y%m%d_%H%M%S)"

if [ ! -x "${RUNNER}" ]; then
    echo "[ERROR] Runner not found or not executable:"
    echo "${RUNNER}"
    exit 1
fi

run_test()
{
    local scenario="$1"
    local planner="$2"
    local label="$3"
    local run_id="${label}_${BATCH_TIME}"

    echo
    echo "================================================================"
    echo "PREVALIDATION"
    echo "scenario = ${scenario}"
    echo "planner  = ${planner}"
    echo "seed     = ${SEED}"
    echo "run_id   = ${run_id}"
    echo "================================================================"

    "${RUNNER}" \
        --planner "${planner}" \
        --scenario "${scenario}" \
        --run-id "${run_id}" \
        --seed "${SEED}" \
        --stack-timeout 180 \
        --planner-timeout 180

    echo
    echo "[PASS] ${scenario} / ${planner}"
}

# Radiation-focused scenario
run_test H_R1_radiation asd pre_HR1_asd_s31
run_test H_R1_radiation tp  pre_HR1_tp_s31
run_test H_R1_radiation aco pre_HR1_aco_s31

# Terrain-focused scenario
run_test H_T1_terrain asd pre_HT1_asd_s31
run_test H_T1_terrain tp  pre_HT1_tp_s31
run_test H_T1_terrain aco pre_HT1_aco_s31

echo
echo "================================================================"
echo "ALL SIX REMAINING PREVALIDATION RUNS PASSED"
echo "Batch: ${BATCH_TIME}"
echo "================================================================"
