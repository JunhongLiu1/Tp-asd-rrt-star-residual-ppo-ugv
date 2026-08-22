#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

WEIGHTS = {
    'radiation': 0.4,
    'terrain': 0.4,
    'execution_time': 0.2,
}
REFERENCE_MARGIN = 1.10


def as_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def load_manifest(path: Path) -> List[Dict[str, str]]:
    with path.open(newline='', encoding='utf-8') as handle:
        return [dict(row) for row in csv.DictReader(handle, delimiter='\t')]


def derive_references(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    successful = [row for row in rows if row['success']]
    if len(successful) != 3:
        raise RuntimeError('Exactly three successful pilot runs are required to calibrate references')

    values = {
        'radiation_reference': max(float(row['executed_radiation_map_cost']) for row in successful),
        'terrain_reference': max(float(row['executed_terrain_cost']) for row in successful),
        'execution_time_reference_s': max(float(row['execution_time_s']) for row in successful),
    }
    for key in list(values):
        values[key] *= REFERENCE_MARGIN
        if values[key] <= 0.0:
            raise RuntimeError(f'Invalid reference {key}: {values[key]}')

    return {
        'version': 'r2_references_candidate_v1',
        'status': 'candidate_from_pilot_not_yet_frozen',
        'scenario': successful[0]['scenario'],
        'seed': successful[0]['seed'],
        'reference_rule': 'max successful pilot value * 1.10',
        'reference_margin': REFERENCE_MARGIN,
        'weights': WEIGHTS,
        **values,
    }


def load_references(path: Path) -> Dict[str, Any]:
    references = json.loads(path.read_text(encoding='utf-8'))
    required = (
        'radiation_reference',
        'terrain_reference',
        'execution_time_reference_s',
    )
    for key in required:
        value = as_float(references.get(key))
        if value is None or value <= 0.0:
            raise RuntimeError(f'Missing or invalid {key} in {path}')
        references[key] = value
    return references


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--mode', choices=('calibrate', 'evaluate'), required=True)
    parser.add_argument('--references')
    parser.add_argument('--radiation-map-stats')
    args = parser.parse_args()

    manifest_path = Path(args.manifest).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, Any]] = []
    for item in load_manifest(manifest_path):
        summary_text = (item.get('summary_json') or '').strip()
        runner_exit_code = int(item.get('runner_exit_code') or 999)
        summary_path = Path(summary_text) if summary_text else None

        if summary_path is None or not summary_path.is_file():
            rows.append({
                'scenario': item.get('scenario', ''),
                'planner_key': item.get('planner_key', ''),
                'planner_name': item.get('planner_key', ''),
                'seed': item.get('seed', ''),
                'runner_exit_code': runner_exit_code,
                'success': False,
                'failure_reason': 'summary.json missing',
                'summary_json': summary_text,
            })
            continue

        data = json.loads(summary_path.read_text(encoding='utf-8'))
        radiation = as_float(data.get('executed_radiation_map_cost'))
        terrain = as_float(data.get('executed_terrain_cost'))
        execution_time = as_float(data.get('execution_time_follower_s'))
        success = (
            as_bool(data.get('success'))
            and runner_exit_code == 0
            and radiation is not None
            and terrain is not None
            and execution_time is not None
            and as_bool(data.get('contact_pass'))
        )
        rows.append({
            'scenario': data.get('scenario', item.get('scenario', '')),
            'planner_key': data.get('planner_key', item.get('planner_key', '')),
            'planner_name': data.get('planner_name', item.get('planner_key', '')),
            'seed': data.get('seed', item.get('seed', '')),
            'runner_exit_code': runner_exit_code,
            'success': success,
            'executed_radiation_map_cost': radiation,
            'executed_terrain_cost': terrain,
            'execution_time_s': execution_time,
            'dose_during_execution_usv': as_float(data.get('dose_during_execution_usv')),
            'executed_path_length_m': as_float(data.get('executed_path_length_m')),
            'planning_time_s': as_float(data.get('planning_time_wall_s')),
            'final_goal_error_m': as_float(data.get('final_goal_error_m')),
            'tracking_rms_error_m': as_float(data.get('tracking_rms_error_m')),
            'contact_pass': as_bool(data.get('contact_pass')),
            'legacy_recorded_score': as_float(data.get('executed_final_coupled_score')),
            'failure_reason': data.get('failure_reason', ''),
            'summary_json': str(summary_path),
        })

    if args.mode == 'calibrate':
        references = derive_references(rows)
        references_path = output_dir / 'normalization_references_candidate.json'
        references_path.write_text(json.dumps(references, indent=2), encoding='utf-8')
    else:
        if not args.references:
            raise RuntimeError('--references is required in evaluate mode')
        references_path = Path(args.references).expanduser().resolve()
        references = load_references(references_path)

    for row in rows:
        row['rank'] = ''
        row['comparison_valid'] = False
        row['radiation_weight'] = WEIGHTS['radiation']
        row['terrain_weight'] = WEIGHTS['terrain']
        row['execution_time_weight'] = WEIGHTS['execution_time']
        row['radiation_reference'] = references['radiation_reference']
        row['terrain_reference'] = references['terrain_reference']
        row['execution_time_reference_s'] = references['execution_time_reference_s']

        if not row.get('success'):
            row['normalized_radiation'] = None
            row['normalized_terrain'] = None
            row['normalized_execution_time'] = None
            row['final_score'] = None
            continue

        r_norm = float(row['executed_radiation_map_cost']) / references['radiation_reference']
        t_norm = float(row['executed_terrain_cost']) / references['terrain_reference']
        e_norm = float(row['execution_time_s']) / references['execution_time_reference_s']
        row['normalized_radiation'] = r_norm
        row['normalized_terrain'] = t_norm
        row['normalized_execution_time'] = e_norm
        row['final_score'] = 100.0 * (
            WEIGHTS['radiation'] * r_norm
            + WEIGHTS['terrain'] * t_norm
            + WEIGHTS['execution_time'] * e_norm
        )

    expected = {'asd', 'tp', 'aco'}
    successful_keys = {str(row.get('planner_key')) for row in rows if row.get('success')}
    comparison_valid = len(rows) == 3 and successful_keys == expected
    successful_rows = [row for row in rows if row.get('success')]
    successful_rows.sort(key=lambda row: float(row['final_score']))
    ranks = {str(row['planner_key']): index for index, row in enumerate(successful_rows, 1)}
    for row in rows:
        row['comparison_valid'] = comparison_valid
        if row.get('success'):
            row['rank'] = ranks[str(row['planner_key'])]
    rows.sort(key=lambda row: (0 if row.get('success') else 1, row.get('rank') or 999))

    stats = None
    if args.radiation_map_stats:
        stats_path = Path(args.radiation_map_stats).expanduser().resolve()
        if stats_path.is_file():
            stats = json.loads(stats_path.read_text(encoding='utf-8'))

    csv_path = output_dir / 'normalized_final_score_comparison.csv'
    json_path = output_dir / 'normalized_final_score_comparison.json'
    fieldnames = [
        'rank', 'comparison_valid', 'scenario', 'planner_key', 'planner_name',
        'seed', 'success', 'runner_exit_code', 'final_score',
        'radiation_weight', 'terrain_weight', 'execution_time_weight',
        'executed_radiation_map_cost', 'radiation_reference', 'normalized_radiation',
        'executed_terrain_cost', 'terrain_reference', 'normalized_terrain',
        'execution_time_s', 'execution_time_reference_s', 'normalized_execution_time',
        'dose_during_execution_usv', 'executed_path_length_m', 'planning_time_s',
        'final_goal_error_m', 'tracking_rms_error_m', 'contact_pass',
        'legacy_recorded_score', 'failure_reason', 'summary_json',
    ]
    with csv_path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        'comparison_valid': comparison_valid,
        'mode': args.mode,
        'ranking_rule': 'lower final_score is better',
        'formula': '100*(0.4*R/R_ref + 0.4*T/T_ref + 0.2*E/E_ref)',
        'references_file': str(references_path),
        'references': references,
        'radiation_map_stats': stats,
        'rows': rows,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')

    print('\n' + '=' * 116)
    print('R2 NORMALIZED FINAL SCORE — PILOT CALIBRATION' if args.mode == 'calibrate' else 'R2 NORMALIZED FINAL SCORE')
    print('Lower final score is better; weights R/T/Time = 0.4/0.4/0.2')
    print('=' * 116)
    print(f"{'RANK':<6}{'PLANNER':<22}{'FINAL':>12}{'R NORM':>12}{'T NORM':>12}{'TIME NORM':>12}{'DOSE uSv':>14}{'SUCCESS':>10}")
    for row in rows:
        def fmt(value, digits=6):
            return 'N/A' if value is None or value == '' else f'{float(value):.{digits}f}'
        print(
            f"{str(row.get('rank', '')):<6}{str(row.get('planner_name', '')):<22}"
            f"{fmt(row.get('final_score')):>12}{fmt(row.get('normalized_radiation')):>12}"
            f"{fmt(row.get('normalized_terrain')):>12}{fmt(row.get('normalized_execution_time')):>12}"
            f"{fmt(row.get('dose_during_execution_usv')):>14}{str(row.get('success')):>10}"
        )
    print('=' * 116)
    print('comparison_valid:', comparison_valid)
    print('references:', references_path)
    if stats:
        print('radiation map >=90 fraction: {:.3%}'.format(stats.get('fraction_ge_90', 0.0)))
        print('radiation map =100 fraction: {:.3%}'.format(stats.get('fraction_eq_100', 0.0)))
    print('CSV: ', csv_path)
    print('JSON:', json_path)
    return 0 if comparison_valid else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        sys.exit(1)
