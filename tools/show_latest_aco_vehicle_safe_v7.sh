#!/usr/bin/env bash
set -eo pipefail

RUN="$(
  find "${HOME}/terrain_radiation_ws/formal_experiments_v3/H_S1_R3_final/aco" \
    -maxdepth 1 -type d \
    -name 'run_aco_vehicle_safe_v7_s31_*' \
    -printf '%T@ %p\n' 2>/dev/null |
  sort -n |
  tail -n 1 |
  cut -d' ' -f2-
)"

if [ -z "${RUN}" ]; then
  echo "[ERROR] No ACO Vehicle-Safe V7 result found"
  exit 1
fi

echo "RUN=${RUN}"
python3 - "${RUN}" <<'PY'
import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
summary_file = run / 'summary.json'
if not summary_file.is_file():
    raise SystemExit(f'Missing: {summary_file}')
summary = json.loads(summary_file.read_text(encoding='utf-8'))

print('=' * 78)
print('ACO VEHICLE-SAFE V7 SUMMARY')
print('=' * 78)
for key in (
    'success',
    'failure_reason',
    'planning_time_wall_s',
    'execution_time_follower_s',
    'executed_path_length_m',
    'executed_radiation_map_cost',
    'executed_terrain_cost',
    'dose_during_execution_usv',
    'final_goal_error_m',
    'tracking_rms_error_m',
    'contact_pass',
):
    print(f'{key}: {summary.get(key)}')

contact_files = list(run.glob('**/*contact*summary*.json'))
if not contact_files:
    print('\nContact summary not found')
    raise SystemExit(0)
contact = json.loads(contact_files[0].read_text(encoding='utf-8'))

print('\n' + '=' * 78)
print('CONTACT ACCEPTANCE')
print('=' * 78)
for key, value in contact.get('acceptance', {}).items():
    print(f'{key}: {value}')
chassis = contact.get('chassis_stats', {})
print('chassis_contact_blocks:', chassis.get('contact_blocks'))
print('chassis_contact_points:', chassis.get('points'))
print('longest_low_support_s:', contact.get('longest_low_support_s'))
print('\nWHEEL P95')
for wheel, stats in contact.get('wheel_stats', {}).items():
    p95 = stats.get('p95_m')
    value = 'N/A' if p95 is None else f'{1000.0 * p95:.3f} mm'
    print(f'{wheel}: {value}')
PY
