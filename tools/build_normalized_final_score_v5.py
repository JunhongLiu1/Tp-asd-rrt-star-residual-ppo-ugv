#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

WEIGHTS = {'radiation': 0.4, 'terrain': 0.4, 'execution_time': 0.2}
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


def spread_percent(values: List[float]) -> Optional[float]:
    if not values:
        return None
    minimum = min(values)
    maximum = max(values)
    if minimum <= 0.0:
        return None
    return 100.0 * (maximum - minimum) / minimum


def derive_references(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    successful = [row for row in rows if row['success']]
    if len(successful) != 3:
        raise RuntimeError('Exactly three successful pilot runs are required')

    references = {
        'radiation_reference': max(float(r['executed_radiation_map_cost']) for r in successful) * REFERENCE_MARGIN,
        'terrain_reference': max(float(r['executed_terrain_cost']) for r in successful) * REFERENCE_MARGIN,
        'execution_time_reference_s': max(float(r['execution_time_s']) for r in successful) * REFERENCE_MARGIN,
    }
    for key, value in references.items():
        if value <= 0.0:
            raise RuntimeError(f'Invalid {key}: {value}')

    return {
        'version': 'r3_references_candidate_v1',
        'status': 'candidate_from_single_seed_pilot_not_frozen',
        'reference_rule': 'max successful pilot value * 1.10',
        'reference_margin': REFERENCE_MARGIN,
        'weights': WEIGHTS,
        **references,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--radiation-map-stats')
    args = parser.parse_args()

    manifest = Path(args.manifest).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, Any]] = []
    for item in load_manifest(manifest):
        summary_text = (item.get('summary_json') or '').strip()
        summary_path = Path(summary_text) if summary_text else None
        runner_exit_code = int(item.get('runner_exit_code') or 999)

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

    references = derive_references(rows)
    references_path = output_dir / 'normalization_references_candidate.json'
    references_path.write_text(json.dumps(references, indent=2), encoding='utf-8')

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

        rn = float(row['executed_radiation_map_cost']) / references['radiation_reference']
        tn = float(row['executed_terrain_cost']) / references['terrain_reference']
        en = float(row['execution_time_s']) / references['execution_time_reference_s']
        row['normalized_radiation'] = rn
        row['normalized_terrain'] = tn
        row['normalized_execution_time'] = en
        row['final_score'] = 100.0 * (0.4 * rn + 0.4 * tn + 0.2 * en)

    expected = {'asd', 'tp', 'aco'}
    successful = [row for row in rows if row.get('success')]
    successful_keys = {str(row.get('planner_key')) for row in successful}
    comparison_valid = len(rows) == 3 and successful_keys == expected
    successful.sort(key=lambda row: float(row['final_score']))
    rank_map = {str(row['planner_key']): i for i, row in enumerate(successful, 1)}
    for row in rows:
        row['comparison_valid'] = comparison_valid
        if row.get('success'):
            row['rank'] = rank_map[str(row['planner_key'])]
    rows.sort(key=lambda row: (0 if row.get('success') else 1, row.get('rank') or 999))

    map_stats = None
    if args.radiation_map_stats:
        stats_path = Path(args.radiation_map_stats).expanduser().resolve()
        if stats_path.is_file():
            map_stats = json.loads(stats_path.read_text(encoding='utf-8'))

    r_values = [float(row['executed_radiation_map_cost']) for row in successful]
    t_values = [float(row['executed_terrain_cost']) for row in successful]
    e_values = [float(row['execution_time_s']) for row in successful]
    radiation_spread = spread_percent(r_values)
    terrain_spread = spread_percent(t_values)
    execution_spread = spread_percent(e_values)

    map_has_high_core = bool(map_stats and map_stats.get('fraction_ge_90', 0.0) > 0.0)
    map_not_over_saturated = bool(map_stats and map_stats.get('fraction_eq_100', 1.0) < 0.10)
    radiation_discrimination_pass = bool(
        radiation_spread is not None and radiation_spread >= 10.0
    )
    pilot_ready_for_multiseed = bool(
        comparison_valid
        and radiation_discrimination_pass
        and map_has_high_core
        and map_not_over_saturated
    )

    csv_path = output_dir / 'normalized_final_score_comparison.csv'
    json_path = output_dir / 'normalized_final_score_comparison.json'
    assessment_path = output_dir / 'r3_pilot_assessment.json'
    fields = [
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
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    assessment = {
        'comparison_valid': comparison_valid,
        'radiation_cost_relative_spread_percent': radiation_spread,
        'terrain_cost_relative_spread_percent': terrain_spread,
        'execution_time_relative_spread_percent': execution_spread,
        'radiation_discrimination_pass_ge_10_percent': radiation_discrimination_pass,
        'map_has_cells_ge_90': map_has_high_core,
        'map_fully_saturated_fraction_below_10_percent': map_not_over_saturated,
        'pilot_ready_for_multiseed_calibration': pilot_ready_for_multiseed,
        'radiation_map_stats': map_stats,
    }
    assessment_path.write_text(json.dumps(assessment, indent=2), encoding='utf-8')

    payload = {
        'comparison_valid': comparison_valid,
        'ranking_rule': 'lower final_score is better',
        'formula': '100*(0.4*R/R_ref + 0.4*T/T_ref + 0.2*E/E_ref)',
        'references_file': str(references_path),
        'references': references,
        'assessment_file': str(assessment_path),
        'assessment': assessment,
        'rows': rows,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')

    def fmt(value, digits=6):
        return 'N/A' if value is None or value == '' else f'{float(value):.{digits}f}'

    print('\n' + '=' * 118)
    print('R3 RADIATION-BARRIER PILOT — NORMALIZED FINAL SCORE')
    print('Lower is better; weights Radiation/Terrain/Time = 0.4/0.4/0.2')
    print('=' * 118)
    print(f"{'RANK':<6}{'PLANNER':<22}{'FINAL':>12}{'RAD COST':>12}{'R NORM':>12}{'T NORM':>12}{'TIME NORM':>12}{'SUCCESS':>10}")
    for row in rows:
        print(
            f"{str(row.get('rank', '')):<6}{str(row.get('planner_name', '')):<22}"
            f"{fmt(row.get('final_score')):>12}{fmt(row.get('executed_radiation_map_cost')):>12}"
            f"{fmt(row.get('normalized_radiation')):>12}{fmt(row.get('normalized_terrain')):>12}"
            f"{fmt(row.get('normalized_execution_time')):>12}{str(row.get('success')):>10}"
        )
    print('=' * 118)
    print('comparison_valid:', comparison_valid)
    print('radiation cost spread: {}%'.format('N/A' if radiation_spread is None else f'{radiation_spread:.3f}'))
    if map_stats:
        print('map >=50: {:.3%}'.format(map_stats.get('fraction_ge_50', 0.0)))
        print('map >=70: {:.3%}'.format(map_stats.get('fraction_ge_70', 0.0)))
        print('map >=90: {:.3%}'.format(map_stats.get('fraction_ge_90', 0.0)))
        print('map =100: {:.3%}'.format(map_stats.get('fraction_eq_100', 0.0)))
    print('pilot_ready_for_multiseed_calibration:', pilot_ready_for_multiseed)
    print('CSV:       ', csv_path)
    print('Assessment:', assessment_path)
    print('References:', references_path)
    return 0 if comparison_valid else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as exc:
        print(f'[ERROR] {exc}', file=sys.stderr)
        sys.exit(1)
