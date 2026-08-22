import os
from glob import glob
from setuptools import setup

package_name = 'radiation_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.json')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'dem',
                'processed'
            ),
            glob('dem/processed/*.npz')
            + glob('dem/processed/*.json')
        ),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*.world')
        ),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='i',
    maintainer_email='i@todo.todo',
    description='Radiation mapping module for terrain-radiation coupled navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terrain_layer_publisher = radiation_mapping.terrain_layer_publisher:main',
            'terrain_3d_impedance_map_node = radiation_mapping.terrain_3d_impedance_map_node:main',
            'husky_vehicle_aware_terrain_node = radiation_mapping.husky_vehicle_aware_terrain_node:main',
            'terrain_query_server = radiation_mapping.terrain_query_server:main',
            'radiation_map_node = radiation_mapping.radiation_map_node:main',
            'radiation_query_service = radiation_mapping.radiation_query_service:main',
            'robot_radiation_dose_monitor = radiation_mapping.robot_radiation_dose_monitor:main',
        ],
    },
)
