#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(
    '/home/i/terrain_radiation_ws/src/'
    'radiation_mapping'
)

sys.path.insert(
    0,
    str(PACKAGE_ROOT)
)

from radiation_mapping.common_cost_model import (  # noqa: E402
    CommonCostModel,
)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--config',
        default=(
            '/home/i/terrain_radiation_ws/src/'
            'radiation_mapping/config/'
            'final_cost_model_v1.json'
        )
    )

    parser.add_argument(
        '--output',
        default=(
            '/home/i/terrain_radiation_ws/'
            'experiment_baselines/'
            'cost_model_v1_0/'
            'validation_report.json'
        )
    )

    args = parser.parse_args()

    model = CommonCostModel(
        args.config
    )

    tests = {}

    tests['low_terrain'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.10,
        dose_rate_usv_h=0.50,
        profile_name='balanced',
        include_time_penalty=False
    )

    tests['high_terrain'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.90,
        dose_rate_usv_h=0.50,
        profile_name='balanced',
        include_time_penalty=False
    )

    tests['low_radiation'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.30,
        dose_rate_usv_h=0.10,
        profile_name='balanced',
        include_time_penalty=False
    )

    tests['high_radiation'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.30,
        dose_rate_usv_h=8.00,
        profile_name='balanced',
        include_time_penalty=False
    )

    tests['base'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.50,
        dose_rate_usv_h=2.00,
        profile_name='balanced',
        include_time_penalty=False
    )

    tests['time_penalised'] = model.evaluate_edge(
        distance_m=1.0,
        terrain_impedance=0.50,
        dose_rate_usv_h=2.00,
        profile_name='balanced',
        include_time_penalty=True
    )

    checks = {
        'profile_count_is_three': (
            len(model.profile_names()) == 3
        ),

        'high_terrain_cost_is_larger': (
            tests['high_terrain']['total_cost']
            > tests['low_terrain']['total_cost']
        ),

        'high_radiation_cost_is_larger': (
            tests['high_radiation']['total_cost']
            > tests['low_radiation']['total_cost']
        ),

        'high_terrain_speed_is_lower': (
            tests['high_terrain'][
                'estimated_speed_m_s'
            ]
            < tests['low_terrain'][
                'estimated_speed_m_s'
            ]
        ),

        'time_penalty_increases_cost': (
            tests['time_penalised']['total_cost']
            > tests['base']['total_cost']
        ),
    }

    passed = all(
        checks.values()
    )

    report = {
        'validation_passed': passed,
        'config': str(
            Path(args.config).resolve()
        ),
        'profiles': list(
            model.profile_names()
        ),
        'reference_length_m': (
            model.reference_length_m
        ),
        'reference_time_s': (
            model.reference_time_s
        ),
        'checks': checks,
        'test_results': tests,
    }

    output_path = Path(
        args.output
    ).expanduser().resolve()

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    with output_path.open(
        'w',
        encoding='utf-8'
    ) as file:
        json.dump(
            report,
            file,
            indent=2
        )

    print()
    print('Final cost-model validation')
    print('---------------------------')

    for profile_name in model.profile_names():
        print(
            'Profile:',
            profile_name
        )

    print()

    for check_name, result in checks.items():
        print(
            f'{check_name}:',
            'PASS' if result else 'FAIL'
        )

    print()
    print(
        'Low terrain speed:',
        round(
            tests['low_terrain'][
                'estimated_speed_m_s'
            ],
            6
        ),
        'm/s'
    )

    print(
        'High terrain speed:',
        round(
            tests['high_terrain'][
                'estimated_speed_m_s'
            ],
            6
        ),
        'm/s'
    )

    print(
        'Base edge cost:',
        round(
            tests['base']['total_cost'],
            8
        )
    )

    print(
        'Time-penalised edge cost:',
        round(
            tests['time_penalised'][
                'total_cost'
            ],
            8
        )
    )

    print()
    print(
        'COST MODEL VALIDATION:',
        'PASS' if passed else 'FAIL'
    )

    print('Report:', output_path)
    print()

    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
