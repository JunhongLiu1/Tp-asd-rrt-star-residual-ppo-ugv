#!/usr/bin/env python3

import ast
import json
import re
from collections import defaultdict
from datetime import datetime
from pathlib import Path


WORKSPACE = Path('/home/i/terrain_radiation_ws')

PACKAGE_ROOT = (
    WORKSPACE
    / 'src'
    / 'radiation_mapping'
)

OUTPUT_DIRECTORY = (
    WORKSPACE
    / 'experiment_baselines'
    / 'legacy_planning_stack_v1'
)

JSON_OUTPUT = (
    OUTPUT_DIRECTORY
    / 'planning_stack_audit.json'
)

TEXT_OUTPUT = (
    OUTPUT_DIRECTORY
    / 'planning_stack_audit.txt'
)


FILE_GROUP_PATTERNS = {
    'followers': re.compile(
        r'follower|controller|path_follow|tracking',
        re.IGNORECASE
    ),

    'planners': re.compile(
        r'rrt|aco|planner|planning',
        re.IGNORECASE
    ),

    'evaluators': re.compile(
        r'evaluator|recorder|metric|comparison|result',
        re.IGNORECASE
    ),

    'terrain': re.compile(
        r'terrain|slope|roughness|impedance|travers',
        re.IGNORECASE
    ),

    'radiation': re.compile(
        r'radiation|dose|hazard|source',
        re.IGNORECASE
    ),
}


COST_PATTERN = re.compile(
    r'\b('
    r'cost|score|weight|objective|fitness|'
    r'edge_cost|path_cost|terrain_cost|'
    r'radiation_cost|dose|impedance|'
    r'smoothness|planning_time|execution_time'
    r')\b',
    re.IGNORECASE
)


TOPIC_PATTERN = re.compile(
    r'["\']'
    r'(/(?:[A-Za-z0-9_]+/?)+)'
    r'["\']'
)


def relative(path):
    try:
        return str(
            path.relative_to(PACKAGE_ROOT)
        )
    except ValueError:
        return str(path)


def read_text(path):
    try:
        return path.read_text(
            encoding='utf-8',
            errors='replace'
        )
    except Exception:
        return ''


def classify_file(path, text):
    combined = (
        path.name
        + '\n'
        + text[:5000]
    )

    groups = []

    for group_name, pattern in (
        FILE_GROUP_PATTERNS.items()
    ):
        if pattern.search(combined):
            groups.append(group_name)

    return groups


def analyse_python(path):
    text = read_text(path)

    result = {
        'path': relative(path),
        'groups': classify_file(path, text),
        'classes': [],
        'functions': [],
        'ros_calls': [],
        'topics': sorted(
            set(
                TOPIC_PATTERN.findall(text)
            )
        ),
        'cost_hits': [],
        'syntax_error': None,
    }

    for line_number, line in enumerate(
        text.splitlines(),
        start=1
    ):
        if COST_PATTERN.search(line):
            result['cost_hits'].append({
                'line': line_number,
                'text': line.strip()[:300],
            })

    try:
        tree = ast.parse(
            text,
            filename=str(path)
        )
    except SyntaxError as error:
        result['syntax_error'] = str(error)
        return result

    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            result['classes'].append({
                'name': node.name,
                'line': node.lineno,
            })

        elif isinstance(
            node,
            (
                ast.FunctionDef,
                ast.AsyncFunctionDef
            )
        ):
            result['functions'].append({
                'name': node.name,
                'line': node.lineno,
            })

        elif isinstance(node, ast.Call):
            function_name = None

            if isinstance(node.func, ast.Attribute):
                function_name = node.func.attr

            elif isinstance(node.func, ast.Name):
                function_name = node.func.id

            if function_name in {
                'create_publisher',
                'create_subscription',
                'create_service',
                'create_client',
                'declare_parameter',
                'get_parameter',
            }:
                result['ros_calls'].append({
                    'name': function_name,
                    'line': getattr(
                        node,
                        'lineno',
                        None
                    ),
                })

    result['classes'].sort(
        key=lambda item: item['line']
    )

    result['functions'].sort(
        key=lambda item: item['line']
    )

    result['ros_calls'].sort(
        key=lambda item: (
            item['line']
            if item['line'] is not None
            else -1
        )
    )

    return result


def extract_console_scripts():
    setup_path = PACKAGE_ROOT / 'setup.py'

    text = read_text(setup_path)

    pattern = re.compile(
        r'["\']'
        r'([^"\']+\s*=\s*'
        r'radiation_mapping\.[^"\']+)'
        r'["\']'
    )

    entries = []

    for match in pattern.findall(text):
        entries.append(
            ' '.join(
                match.split()
            )
        )

    return sorted(set(entries))


def extract_launch_executables():
    launch_directory = (
        PACKAGE_ROOT
        / 'launch'
    )

    results = []

    for path in sorted(
        launch_directory.glob('*.py')
    ):
        text = read_text(path)

        executable_matches = re.findall(
            r'executable\s*=\s*'
            r'["\']([^"\']+)["\']',
            text
        )

        package_matches = re.findall(
            r'package\s*=\s*'
            r'["\']([^"\']+)["\']',
            text
        )

        results.append({
            'path': relative(path),
            'executables': sorted(
                set(executable_matches)
            ),
            'packages': sorted(
                set(package_matches)
            ),
            'topics': sorted(
                set(
                    TOPIC_PATTERN.findall(text)
                )
            ),
        })

    return results


def build_summary(python_files):
    grouped_files = defaultdict(list)

    all_topics = defaultdict(list)

    cost_functions = []

    for item in python_files:
        for group in item['groups']:
            grouped_files[group].append(
                item['path']
            )

        for topic in item['topics']:
            all_topics[topic].append(
                item['path']
            )

        cost_function_names = [
            function
            for function in item['functions']
            if COST_PATTERN.search(
                function['name']
            )
        ]

        if (
            cost_function_names
            or item['cost_hits']
        ):
            cost_functions.append({
                'path': item['path'],
                'functions': cost_function_names,
                'hit_count': len(
                    item['cost_hits']
                ),
                'first_hits': (
                    item['cost_hits'][:15]
                ),
            })

    return {
        'grouped_files': {
            key: sorted(set(value))
            for key, value in grouped_files.items()
        },
        'topics': {
            topic: sorted(set(files))
            for topic, files in sorted(
                all_topics.items()
            )
        },
        'cost_related_files': cost_functions,
    }


def write_text_report(report):
    lines = []

    lines.append(
        'PLANNING STACK AUDIT'
    )
    lines.append(
        '=' * 70
    )
    lines.append(
        f"Created: {report['created_at']}"
    )
    lines.append(
        f"Package: {report['package_root']}"
    )
    lines.append('')

    lines.append(
        'CONSOLE SCRIPTS'
    )
    lines.append(
        '-' * 70
    )

    for entry in report['console_scripts']:
        lines.append(entry)

    lines.append('')
    lines.append(
        'CANDIDATE FILE GROUPS'
    )
    lines.append(
        '-' * 70
    )

    for group in (
        'followers',
        'planners',
        'evaluators',
        'terrain',
        'radiation',
    ):
        lines.append(
            f'[{group.upper()}]'
        )

        files = (
            report['summary']
            ['grouped_files']
            .get(group, [])
        )

        if not files:
            lines.append('  No files detected.')
        else:
            for file_path in files:
                lines.append(
                    f'  {file_path}'
                )

        lines.append('')

    lines.append(
        'ROS TOPICS DETECTED'
    )
    lines.append(
        '-' * 70
    )

    for topic, files in (
        report['summary']
        ['topics']
        .items()
    ):
        lines.append(topic)

        for file_path in files:
            lines.append(
                f'  - {file_path}'
            )

    lines.append('')
    lines.append(
        'COST-RELATED FILES AND FUNCTIONS'
    )
    lines.append(
        '-' * 70
    )

    for item in (
        report['summary']
        ['cost_related_files']
    ):
        lines.append(
            f"FILE: {item['path']}"
        )

        if item['functions']:
            for function in item['functions']:
                lines.append(
                    '  FUNCTION: '
                    f"{function['name']} "
                    f"(line {function['line']})"
                )

        lines.append(
            f"  COST HITS: "
            f"{item['hit_count']}"
        )

        for hit in item['first_hits']:
            lines.append(
                f"    L{hit['line']}: "
                f"{hit['text']}"
            )

        lines.append('')

    lines.append(
        'LAUNCH FILE EXECUTABLES'
    )
    lines.append(
        '-' * 70
    )

    for launch in report[
        'launch_files'
    ]:
        lines.append(
            f"FILE: {launch['path']}"
        )

        lines.append(
            '  PACKAGES: '
            + ', '.join(
                launch['packages']
            )
        )

        lines.append(
            '  EXECUTABLES: '
            + ', '.join(
                launch['executables']
            )
        )

        if launch['topics']:
            lines.append(
                '  TOPICS: '
                + ', '.join(
                    launch['topics']
                )
            )

        lines.append('')

    TEXT_OUTPUT.write_text(
        '\n'.join(lines),
        encoding='utf-8'
    )


def main():
    OUTPUT_DIRECTORY.mkdir(
        parents=True,
        exist_ok=True
    )

    search_directories = [
        PACKAGE_ROOT
        / 'radiation_mapping',

        PACKAGE_ROOT
        / 'launch',

        PACKAGE_ROOT
        / 'tools',
    ]

    python_paths = []

    for directory in search_directories:
        if directory.is_dir():
            python_paths.extend(
                directory.rglob('*.py')
            )

    python_paths = sorted(
        set(python_paths)
    )

    python_files = [
        analyse_python(path)
        for path in python_paths
    ]

    report = {
        'created_at': (
            datetime.now().isoformat()
        ),
        'package_root': str(
            PACKAGE_ROOT
        ),
        'python_file_count': len(
            python_files
        ),
        'console_scripts': (
            extract_console_scripts()
        ),
        'launch_files': (
            extract_launch_executables()
        ),
        'python_files': python_files,
        'summary': build_summary(
            python_files
        ),
    }

    with JSON_OUTPUT.open(
        'w',
        encoding='utf-8'
    ) as file:
        json.dump(
            report,
            file,
            indent=2
        )

    write_text_report(report)

    print()
    print('Planning stack audit complete')
    print('-----------------------------')
    print(
        'Python files:',
        report['python_file_count']
    )
    print(
        'Console scripts:',
        len(report['console_scripts'])
    )
    print(
        'Detected topics:',
        len(
            report['summary']['topics']
        )
    )
    print(
        'Cost-related files:',
        len(
            report['summary']
            ['cost_related_files']
        )
    )
    print('JSON:', JSON_OUTPUT)
    print('Text:', TEXT_OUTPUT)
    print()


if __name__ == '__main__':
    main()
