#!/usr/bin/env python3

import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

from datetime import datetime
from pathlib import Path
from urllib.parse import unquote, urlparse


WORKSPACE = Path('/home/i/terrain_radiation_ws')

PACKAGE_DIR = (
    WORKSPACE
    / 'src'
    / 'radiation_mapping'
)

PROCESSED_DIR = (
    PACKAGE_DIR
    / 'dem'
    / 'processed'
)

WORLD_DIR = (
    PACKAGE_DIR
    / 'worlds'
)

OUTPUT_DIR = (
    WORKSPACE
    / 'experiment_baselines'
    / 'terrain_v1_0'
)


LEVEL_CONFIG = {
    'easy': {
        'world_candidates': [
            WORLD_DIR / 'module34_easy.world',
            WORLD_DIR / 'module33_dem_terrain_easy.world',
            WORLD_DIR / 'module33_dem_inspired_terrain_513.world',
        ],
        'metadata_candidates': [
            PROCESSED_DIR / 'dem_terrain_easy_metadata.json',
            PROCESSED_DIR / 'module33_dem_inspired_metadata_513.json',
        ],
        'safe_spawn': {
            'x': -5.543,
            'y': -3.281,
            'z': 1.106,
            'yaw': 0.0,
        },
    },

    'medium': {
        'world_candidates': [
            WORLD_DIR / 'module34_medium.world',
            WORLD_DIR / 'module33_dem_terrain_medium.world',
        ],
        'metadata_candidates': [
            PROCESSED_DIR / 'dem_terrain_medium_metadata.json',
        ],
        'safe_spawn': {
            'x': 5.618,
            'y': 6.973,
            'z': 0.397,
            'yaw': 0.0,
        },
    },

    'hard': {
        'world_candidates': [
            WORLD_DIR / 'module34_hard.world',
            WORLD_DIR / 'module33_dem_terrain_hard.world',
            WORLD_DIR / 'module33_dem_terrain_hard_final.world',
        ],
        'metadata_candidates': [
            PROCESSED_DIR / 'dem_terrain_hard_metadata.json',
            PROCESSED_DIR / 'dem_terrain_hard_final_metadata.json',
        ],
        'safe_spawn': {
            'x': 5.134,
            'y': 5.977,
            'z': 0.448,
            'yaw': 0.0,
        },
    },
}


def first_existing(candidates, description):
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()

    candidate_text = '\n'.join(
        f'  - {candidate}'
        for candidate in candidates
    )

    raise FileNotFoundError(
        f'No valid {description} was found:\n'
        f'{candidate_text}'
    )


def calculate_sha256(path):
    digest = hashlib.sha256()

    with path.open('rb') as file:
        while True:
            block = file.read(1024 * 1024)

            if not block:
                break

            digest.update(block)

    return digest.hexdigest()


def copy_file(source, destination):
    destination.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    shutil.copy2(
        source,
        destination
    )

    return destination.resolve()


def uri_to_local_path(uri_text):
    text = uri_text.strip()

    if text.startswith('file://'):
        parsed = urlparse(text)

        return Path(
            unquote(parsed.path)
        )

    possible_path = Path(text).expanduser()

    if possible_path.is_absolute():
        return possible_path

    return None


def freeze_world(
    source_world,
    frozen_world,
    frozen_heightmap,
    world_assets_directory
):
    tree = ET.parse(source_world)
    root = tree.getroot()

    heightmap_uris = root.findall(
        './/heightmap/uri'
    )

    if not heightmap_uris:
        raise RuntimeError(
            f'No heightmap URI found in: {source_world}'
        )

    # Always force the copied world to use the frozen heightmap.
    for uri_element in heightmap_uris:
        uri_element.text = frozen_heightmap.as_uri()

    # Copy other locally referenced files such as textures.
    for uri_element in root.findall('.//uri'):
        if uri_element in heightmap_uris:
            continue

        if not uri_element.text:
            continue

        local_path = uri_to_local_path(
            uri_element.text
        )

        if local_path is None:
            # model:// and package:// references are retained.
            continue

        local_path = local_path.expanduser()

        if not local_path.is_file():
            continue

        extension = local_path.suffix.lower()

        if extension not in {
            '.png',
            '.jpg',
            '.jpeg',
            '.dae',
            '.stl',
            '.obj',
            '.material',
        }:
            continue

        frozen_asset = (
            world_assets_directory
            / local_path.name
        )

        copy_file(
            local_path,
            frozen_asset
        )

        uri_element.text = (
            frozen_asset.resolve().as_uri()
        )

    frozen_world.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    tree.write(
        frozen_world,
        encoding='utf-8',
        xml_declaration=True
    )


def resolve_heightmap(metadata):
    value = metadata.get(
        'output_heightmap',
        ''
    )

    if not value:
        raise KeyError(
            'Terrain metadata does not contain '
            'output_heightmap.'
        )

    path = Path(value).expanduser()

    if not path.is_absolute():
        path = (
            PROCESSED_DIR
            / path
        )

    if not path.is_file():
        raise FileNotFoundError(
            f'Heightmap not found: {path}'
        )

    return path.resolve()


def extract_summary(metadata):
    gazebo = metadata.get(
        'gazebo',
        {}
    )

    slope = gazebo.get(
        'slope_statistics',
        {}
    )

    return {
        'terrain_size_x_m': gazebo.get(
            'target_x_m'
        ),
        'terrain_size_y_m': gazebo.get(
            'target_y_m'
        ),
        'requested_relief_m': gazebo.get(
            'requested_relief_m'
        ),
        'final_relief_m': gazebo.get(
            'final_relief_m'
        ),
        'vertical_factor': gazebo.get(
            'vertical_factor'
        ),
        'slope_mean_deg': slope.get(
            'slope_mean_deg'
        ),
        'slope_95_deg': slope.get(
            'slope_95_deg'
        ),
        'slope_max_deg': slope.get(
            'slope_max_deg'
        ),
    }


def main():
    if OUTPUT_DIR.exists():
        backup_directory = OUTPUT_DIR.with_name(
            OUTPUT_DIR.name
            + '_backup_'
            + datetime.now().strftime(
                '%Y%m%d_%H%M%S'
            )
        )

        shutil.move(
            str(OUTPUT_DIR),
            str(backup_directory)
        )

        print(
            'Existing baseline moved to:',
            backup_directory
        )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    manifest = {
        'baseline_name': 'terrain_v1_0',
        'created_at': datetime.now().isoformat(),
        'workspace': str(WORKSPACE),
        'description': (
            'Frozen DEM-inspired terrain baseline for '
            'ASD-RRT*, Time-Penalised ASD-RRT* and ACO.'
        ),
        'coordinate_frame': 'map/world',
        'terrain_layer_resolution_m': 0.10,
        'terrain_cost_model': {
            'formula': (
                '0.75 * slope_cost '
                '+ 0.25 * roughness_cost'
            ),
            'slope_weight': 0.75,
            'roughness_weight': 0.25,
        },
        'levels': {},
    }

    for level, configuration in LEVEL_CONFIG.items():
        print()
        print(
            f'Freezing {level.upper()} terrain'
        )
        print('-' * 40)

        source_world = first_existing(
            configuration[
                'world_candidates'
            ],
            f'{level} world'
        )

        source_metadata = first_existing(
            configuration[
                'metadata_candidates'
            ],
            f'{level} terrain metadata'
        )

        with source_metadata.open(
            'r',
            encoding='utf-8'
        ) as file:
            terrain_metadata = json.load(file)

        source_heightmap = resolve_heightmap(
            terrain_metadata
        )

        source_layers = (
            PROCESSED_DIR
            / f'terrain_layers_{level}.npz'
        )

        source_layers_metadata = (
            PROCESSED_DIR
            / f'terrain_layers_{level}_metadata.json'
        )

        if not source_layers.is_file():
            raise FileNotFoundError(
                f'Missing terrain layers: '
                f'{source_layers}'
            )

        if not source_layers_metadata.is_file():
            raise FileNotFoundError(
                'Missing terrain-layer metadata: '
                f'{source_layers_metadata}'
            )

        level_directory = (
            OUTPUT_DIR
            / level
        )

        world_assets_directory = (
            level_directory
            / 'world_assets'
        )

        frozen_heightmap = copy_file(
            source_heightmap,
            level_directory
            / 'heightmap.png'
        )

        frozen_terrain_metadata = copy_file(
            source_metadata,
            level_directory
            / 'terrain_metadata.json'
        )

        frozen_layers = copy_file(
            source_layers,
            level_directory
            / 'terrain_layers.npz'
        )

        frozen_layers_metadata = copy_file(
            source_layers_metadata,
            level_directory
            / 'terrain_layers_metadata.json'
        )

        frozen_world = (
            level_directory
            / 'terrain.world'
        )

        freeze_world(
            source_world=source_world,
            frozen_world=frozen_world,
            frozen_heightmap=frozen_heightmap,
            world_assets_directory=(
                world_assets_directory
            ),
        )

        preview_directory = (
            level_directory
            / 'previews'
        )

        preview_files = []

        for preview_source in sorted(
            PROCESSED_DIR.glob(
                f'terrain_layers_{level}_*.png'
            )
        ):
            preview_destination = copy_file(
                preview_source,
                preview_directory
                / preview_source.name
            )

            preview_files.append(
                str(
                    preview_destination.relative_to(
                        OUTPUT_DIR
                    )
                )
            )

        manifest['levels'][level] = {
            'source_files': {
                'world': str(source_world),
                'heightmap': str(
                    source_heightmap
                ),
                'terrain_metadata': str(
                    source_metadata
                ),
                'terrain_layers': str(
                    source_layers
                ),
                'terrain_layers_metadata': str(
                    source_layers_metadata
                ),
            },
            'frozen_files': {
                'world': str(
                    frozen_world.resolve().relative_to(
                        OUTPUT_DIR
                    )
                ),
                'heightmap': str(
                    frozen_heightmap.relative_to(
                        OUTPUT_DIR
                    )
                ),
                'terrain_metadata': str(
                    frozen_terrain_metadata.relative_to(
                        OUTPUT_DIR
                    )
                ),
                'terrain_layers': str(
                    frozen_layers.relative_to(
                        OUTPUT_DIR
                    )
                ),
                'terrain_layers_metadata': str(
                    frozen_layers_metadata.relative_to(
                        OUTPUT_DIR
                    )
                ),
                'previews': preview_files,
            },
            'safe_spawn': configuration[
                'safe_spawn'
            ],
            'terrain_summary': extract_summary(
                terrain_metadata
            ),
        }

        print('Source world:', source_world)
        print(
            'Source heightmap:',
            source_heightmap
        )
        print('Frozen directory:', level_directory)

    # Add file hashes, excluding the manifest itself.
    file_hashes = {}

    for file_path in sorted(
        OUTPUT_DIR.rglob('*')
    ):
        if not file_path.is_file():
            continue

        relative_path = file_path.relative_to(
            OUTPUT_DIR
        )

        file_hashes[
            str(relative_path)
        ] = calculate_sha256(
            file_path
        )

    manifest['file_sha256'] = file_hashes

    manifest_path = (
        OUTPUT_DIR
        / 'baseline_manifest.json'
    )

    with manifest_path.open(
        'w',
        encoding='utf-8'
    ) as file:
        json.dump(
            manifest,
            file,
            indent=2
        )

    checksum_files = sorted(
        file_path
        for file_path in OUTPUT_DIR.rglob('*')
        if (
            file_path.is_file()
            and file_path.name != 'SHA256SUMS'
        )
    )

    checksum_path = (
        OUTPUT_DIR
        / 'SHA256SUMS'
    )

    with checksum_path.open(
        'w',
        encoding='utf-8'
    ) as file:
        for file_path in checksum_files:
            relative_path = file_path.relative_to(
                OUTPUT_DIR
            )

            file.write(
                f'{calculate_sha256(file_path)}  '
                f'{relative_path}\n'
            )

    print()
    print('=' * 60)
    print('TERRAIN BASELINE FREEZE COMPLETE')
    print('=' * 60)
    print('Output:', OUTPUT_DIR)
    print('Manifest:', manifest_path)
    print('Checksums:', checksum_path)
    print()


if __name__ == '__main__':
    main()
